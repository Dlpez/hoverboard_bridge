"""Puente ROS 2 <-> hoverboard (firmware hack-FOC por USART), en Python.

Reemplaza a ros2_control + hoverboard_hardware_interface + diff_drive_controller (ese driver
C++ no está en el repo). Mismos topics que diff_drive_controller, así el resto no cambia:

Entrada:  cmd_vel (geometry_msgs/Twist): v [m/s] y w [rad/s] del robot
Salidas:  /odom (nav_msgs/Odometry), TF odom -> base_footprint, /joint_states (ruedas),
          /hoverboard/bateria (std_msgs/Float32, volts)

Cómo controla: el firmware recibe (speed, steer) y devuelve las RPM y los contadores Hall de cada
rueda. Con la calibración (ros2 run hoverboard_bridge calibrar) se sabe cómo (speed, steer) mueve
cada rueda; el puente pide las RPM de cada rueda con un PI que corrige con las RPM medidas.
Odometría: con los contadores Hall (ticks_per_rev por vuelta de rueda).
Seguridad: sin cmd_vel por cmd_timeout -> para; sin feedback del hoverboard -> para y avisa.
"""
import math
import os

import numpy as np
import rclpy
import serial
import yaml
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from hoverboard_bridge.protocolo import FeedbackParser, delta_int16, make_command


class HoverboardBridge(Node):
    def __init__(self):
        super().__init__('hoverboard_bridge')
        p = self.declare_parameter
        p('port', '/dev/ttyUSB0')
        p('baud_rate', 115200)
        p('calibration_file', os.path.expanduser('~/.ros/hoverboard_calibracion.yaml'))
        p('wheel_radius', 0.08255)          # [m] rueda de hoverboard 6.5"
        # [m] separación EFECTIVA para la odometría. Física (centro a centro de los neumáticos,
        # medida): 0.32 m. Calibrada con el lidar (ICP) girando 1 vuelta en el lugar el 10-11-2026:
        # odom 372.5° vs real 344.7° con 0.32 -> 0.32 * 1.081 = 0.346 (las ruedas patinan al girar).
        p('wheel_separation', 0.346)
        p('ticks_per_rev', 90.0)            # pasos Hall por vuelta de rueda
        p('max_command', 400)               # tope de |speed| y |steer| enviados (de 1000)
        p('max_linear', 1.67)               # [m/s] tope duro (6 km/h), venga de donde venga
        p('max_angular', 0.4)               # [rad/s] giro lento (~23°/s)
        # Rampas: acelerar suave (sin "despegue") y frenar más fuerte (para no pasarse).
        p('linear_accel', 0.5)              # [m/s²] 0 -> 6 km/h en ~3.3 s
        p('linear_decel', 1.0)              # [m/s²] suave; 6 km/h -> 0 en ~1.7 s (~1.4 m)
        p('angular_accel', 0.8)             # [rad/s²]
        p('angular_decel', 1.2)             # [rad/s²]
        # PI propio sobre las RPM: APAGADO. El firmware ya controla velocidad (comando 80 -> 80 RPM)
        # y un segundo lazo encima hacía que las ruedas "buscaran" la posición al frenar (temblor).
        p('kp', 0.0)
        p('ki', 0.0)
        p('integral_limit', 40.0)           # [RPM*s]
        p('cmd_timeout', 0.5)               # [s] sin cmd_vel -> parar
        p('feedback_timeout', 0.5)          # [s] sin feedback -> parar
        p('stall_reopen', 1.5)              # [s] sin feedback con el puerto abierto -> reabrir
                                            # (el CH340 se "cuelga" con el ruido de los motores)
        p('rate', 50.0)                     # [Hz]
        p('odom_frame', 'odom')
        p('base_frame', 'base_footprint')
        p('publish_tf', True)
        p('left_joint', 'left_wheel_joint')
        p('right_joint', 'right_wheel_joint')
        # Batería 10S (Li-ion): 42 V llena, ~36 V nominal, cortar ~33 V.
        p('bat_full', 42.0)
        p('bat_empty', 33.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.r = g('wheel_radius')
        self.sep = g('wheel_separation')
        self.tpr = g('ticks_per_rev')
        self.max_cmd = g('max_command')
        self.kp, self.ki, self.ilim = g('kp'), g('ki'), g('integral_limit')
        self.cmd_timeout = g('cmd_timeout')
        self.max_v, self.max_w = g('max_linear'), g('max_angular')
        self.acc_v, self.dec_v = g('linear_accel'), g('linear_decel')
        self.acc_w, self.dec_w = g('angular_accel'), g('angular_decel')
        self.v_ramp = self.w_ramp = 0.0     # velocidad que se está pidiendo, ya con rampa
        self.fb_timeout = g('feedback_timeout')
        self.stall_reopen = g('stall_reopen')
        self.port_opened_at = None
        self.odom_frame, self.base_frame = g('odom_frame'), g('base_frame')
        self.publish_tf = g('publish_tf')
        self.joints = [g('left_joint'), g('right_joint')]
        self.bat_full, self.bat_empty = g('bat_full'), g('bat_empty')
        self.last_marker = 0.0

        cal_path = g('calibration_file')
        if not os.path.exists(cal_path):
            raise RuntimeError(f'Falta la calibración {cal_path}: '
                               'correr "ros2 run hoverboard_bridge calibrar" (ruedas en el aire)')
        with open(cal_path) as f:
            cal = yaml.safe_load(f)
        self.rpm_sign = np.array([cal['rpm_sign_left'], cal['rpm_sign_right']], dtype=float)
        self.tick_sign = np.array([cal['tick_sign_left'], cal['tick_sign_right']], dtype=float)
        mix = np.array(cal['mix'], dtype=float).reshape(2, 2)
        # swap_wheels: la rueda que el firmware llama "izquierda" es la derecha del robot (en Milo
        # el giro salía invertido con el avance correcto). Se intercambian filas y signos para
        # que control Y odometría usen la rueda física correcta.
        self.swap = bool(cal.get('swap_wheels', False))
        if self.swap:
            self.rpm_sign = self.rpm_sign[::-1].copy()
            self.tick_sign = self.tick_sign[::-1].copy()
            mix = mix[::-1].copy()
        self.mix_inv = np.linalg.inv(mix)
        self.get_logger().info(f'Calibración: {cal_path} (swap_wheels={self.swap})')

        self.port, self.baud = g('port'), g('baud_rate')
        self.ser = None
        self.next_reconnect = 0.0
        self.parser = FeedbackParser()
        self.open_port()

        # Estado
        self.target = (0.0, 0.0)            # v, w pedidos
        self.last_cmd = None
        self.last_fb = None
        self.fb = None
        self.prev_cnt = None
        self.integ = np.zeros(2)
        self.f_meas = np.zeros(2)           # RPM hacia adelante medidas [izq, der]
        self.wheel_pos = np.zeros(2)        # [rad]
        self.x = self.y = self.th = 0.0
        self.last_bat_log = 0.0

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.bat_pub = self.create_publisher(Float32, '/hoverboard/bateria', 10)
        # Textos para RViz (display Marker en este topic): batería sobre el robot y velocidad
        # de cada rueda sobre su rueda.
        self.marker_pub = self.create_publisher(Marker, '/hoverboard/markers', 10)
        self.tf = TransformBroadcaster(self)
        self.dt = 1.0 / g('rate')
        self.create_timer(self.dt, self.step)

    def open_port(self):
        """Abre el puerto. Si el USB se desconecta (ruido de los motores), se reintenta cada 1 s."""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.port_opened_at = self.get_clock().now()
            self.parser = FeedbackParser()
            self.prev_cnt = None  # el contador pudo avanzar mientras no leíamos
            self.get_logger().info(f'Hoverboard en {self.port}')
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().warn(f'No se pudo abrir {self.port}: {e}', throttle_duration_sec=5.0)

    def drop_port(self, e):
        self.get_logger().error(f'Puerto serie perdido ({e}); reintentando cada 1 s')
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.integ[:] = 0.0

    @staticmethod
    def ramp(cur, tgt, acc, dec, dt):
        """Acerca cur a tgt: con `acc` si se aleja de 0 (acelera), con `dec` si frena."""
        speeding_up = tgt * cur >= 0.0 and abs(tgt) > abs(cur)
        step = (acc if speeding_up else dec) * dt
        return min(tgt, cur + step) if tgt > cur else max(tgt, cur - step)

    def on_cmd(self, msg):
        self.target = (msg.linear.x, msg.angular.z)
        self.last_cmd = self.get_clock().now()

    def secs_since(self, t):
        return (self.get_clock().now() - t).nanoseconds * 1e-9 if t is not None else 1e9

    # ---------------- lectura ----------------
    def read_feedback(self):
        for fb in self.parser.feed(self.ser.read(1024)):
            self.fb = fb
            self.last_fb = self.get_clock().now()
            cnt = np.array([fb.cnt_r, fb.cnt_l] if self.swap else [fb.cnt_l, fb.cnt_r])
            if self.prev_cnt is not None:
                d = np.array([delta_int16(cnt[0], self.prev_cnt[0]),
                              delta_int16(cnt[1], self.prev_cnt[1])], dtype=float)
                self.integrate(self.tick_sign * d * 2.0 * math.pi / self.tpr)
            self.prev_cnt = cnt
            rpm = [fb.rpm_r, fb.rpm_l] if self.swap else [fb.rpm_l, fb.rpm_r]
            self.f_meas = self.rpm_sign * np.array(rpm, dtype=float)

    def integrate(self, dphi):
        """dphi = ángulo girado por cada rueda [rad] (+ = hacia adelante)."""
        self.wheel_pos += dphi
        dl, dr = dphi * self.r
        ds = 0.5 * (dl + dr)
        dth = (dr - dl) / self.sep
        self.x += ds * math.cos(self.th + 0.5 * dth)
        self.y += ds * math.sin(self.th + 0.5 * dth)
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))

    # ---------------- control ----------------
    def step(self):
        now = self.get_clock().now()
        if self.ser is None:
            t = now.nanoseconds * 1e-9
            if t >= self.next_reconnect:
                self.next_reconnect = t + 1.0
                self.open_port()
            if self.ser is None:
                self.f_meas[:] = 0.0
                self.publish(now)
                return
        try:
            self.read_feedback()
        except (serial.SerialException, OSError) as e:
            self.drop_port(e)
            return
        # Puerto abierto pero mudo: el adaptador se colgó sin desconectarse. Reabrirlo.
        last_ok = self.last_fb if self.last_fb is not None else self.port_opened_at
        if self.secs_since(last_ok) > self.stall_reopen and \
                self.secs_since(self.port_opened_at) > self.stall_reopen:
            self.drop_port('sin datos %.1f s' % self.secs_since(last_ok))
            return

        v, w = self.target
        if self.secs_since(self.last_cmd) > self.cmd_timeout:
            v = w = 0.0
        v = max(-self.max_v, min(self.max_v, v))
        w = max(-self.max_w, min(self.max_w, w))
        fb_ok = self.secs_since(self.last_fb) <= self.fb_timeout
        if not fb_ok:
            v = w = 0.0
            self.v_ramp = self.w_ramp = 0.0
        # Rampa de aceleración / frenado.
        self.v_ramp = self.ramp(self.v_ramp, v, self.acc_v, self.dec_v, self.dt)
        self.w_ramp = self.ramp(self.w_ramp, w, self.acc_w, self.dec_w, self.dt)
        v, w = self.v_ramp, self.w_ramp
        if not fb_ok:
            self.get_logger().warn('Sin feedback del hoverboard: parado',
                                   throttle_duration_sec=2.0)

        k = 60.0 / (2.0 * math.pi * self.r)  # m/s -> RPM
        f_tgt = np.array([(v - w * self.sep / 2.0) * k, (v + w * self.sep / 2.0) * k])

        if not fb_ok or (abs(v) < 1e-3 and abs(w) < 1e-3):
            self.integ[:] = 0.0
            speed = steer = 0
        else:
            e = f_tgt - self.f_meas
            self.integ = np.clip(self.integ + e * self.dt, -self.ilim, self.ilim)
            f_cmd = f_tgt + self.kp * e + self.ki * self.integ
            speed, steer = self.mix_inv @ f_cmd
            speed = int(np.clip(round(speed), -self.max_cmd, self.max_cmd))
            steer = int(np.clip(round(steer), -self.max_cmd, self.max_cmd))
        try:
            self.ser.write(make_command(steer, speed))
        except (serial.SerialException, OSError) as e:
            self.drop_port(e)

        self.publish(now)

    def publish(self, now):
        stamp = now.to_msg()
        q_z, q_w = math.sin(self.th / 2.0), math.cos(self.th / 2.0)
        vel = self.f_meas * 2.0 * math.pi / 60.0 * self.r  # m/s de cada rueda
        v_meas = 0.5 * (vel[0] + vel[1])
        w_meas = (vel[1] - vel[0]) / self.sep

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = q_z
        od.pose.pose.orientation.w = q_w
        od.twist.twist.linear.x = v_meas
        od.twist.twist.angular.z = w_meas
        for i, c in zip((0, 7, 14, 21, 28, 35), (1e-3, 1e-3, 1e6, 1e6, 1e6, 5e-3)):
            od.pose.covariance[i] = c
            od.twist.covariance[i] = c
        self.odom_pub.publish(od)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = q_z
            t.transform.rotation.w = q_w
            self.tf.sendTransform(t)

        js = JointState()
        js.header.stamp = stamp
        js.name = self.joints
        js.position = [float(p) for p in self.wheel_pos]
        js.velocity = [float(x) for x in self.f_meas * 2.0 * math.pi / 60.0]
        self.js_pub.publish(js)

        t_s = now.nanoseconds * 1e-9
        if t_s - self.last_marker > 0.5:
            self.last_marker = t_s
            self.publish_battery_marker(stamp)
            self.publish_wheel_markers(stamp)
        if self.fb is not None:
            self.bat_pub.publish(Float32(data=float(self.fb.bat)))
            if t_s - self.last_bat_log > 60.0:
                self.last_bat_log = t_s
                self.get_logger().info(f'Batería {self.fb.bat:.2f} V, placa {self.fb.temp:.1f} °C')

    def publish_battery_marker(self, stamp):
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.base_frame
        m.ns, m.id = 'bateria', 0
        m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.z = 1.9          # sobre la cabeza de Milo
        m.pose.orientation.w = 1.0
        m.scale.z = 0.18                 # alto del texto [m]
        m.color.a = 1.0
        ok = self.fb is not None and self.ser is not None and \
            self.secs_since(self.last_fb) <= self.fb_timeout
        if not ok:
            m.text = 'Bateria: SIN DATOS'
            m.color.r, m.color.g, m.color.b = 0.6, 0.6, 0.6
        else:
            v = self.fb.bat
            pct = max(0.0, min(1.0, (v - self.bat_empty) / (self.bat_full - self.bat_empty)))
            m.text = f'Bateria {v:.1f} V ({pct * 100:.0f}%)'
            # verde -> amarillo -> rojo
            m.color.r = min(1.0, 2.0 * (1.0 - pct))
            m.color.g = min(1.0, 2.0 * pct)
            m.color.b = 0.0
        self.marker_pub.publish(m)

    def publish_wheel_markers(self, stamp):
        ok = self.ser is not None and self.secs_since(self.last_fb) <= self.fb_timeout
        rpm = self.f_meas if ok else np.zeros(2)
        mps = rpm * 2.0 * math.pi / 60.0 * self.r
        for i, (nombre, y) in enumerate((('Izq', self.sep / 2.0), ('Der', -self.sep / 2.0))):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.base_frame
            m.ns, m.id = 'ruedas', i
            m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            m.pose.position.y = y
            m.pose.position.z = 0.55
            m.pose.orientation.w = 1.0
            m.scale.z = 0.09
            m.color.a = 1.0
            if not ok:
                m.text = f'{nombre}: --'
                m.color.r = m.color.g = m.color.b = 0.6
            else:
                m.text = f'{nombre}: {mps[i]:+.2f} m/s\n{rpm[i]:+.0f} rpm'
                # celeste adelante, naranjo atrás, blanco quieto
                if abs(rpm[i]) < 1.0:
                    m.color.r = m.color.g = m.color.b = 1.0
                elif rpm[i] > 0:
                    m.color.r, m.color.g, m.color.b = 0.3, 0.8, 1.0
                else:
                    m.color.r, m.color.g, m.color.b = 1.0, 0.6, 0.1
            self.marker_pub.publish(m)

    def stop(self):
        if self.ser is None:
            return
        for _ in range(5):
            try:
                self.ser.write(make_command(0, 0))
            except Exception:
                pass
        self.ser.close()


def main():
    rclpy.init()
    node = HoverboardBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
