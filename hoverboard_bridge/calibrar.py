"""Calibración del hoverboard: CON LAS RUEDAS EN EL AIRE.

Manda dos pruebas cortas y mide qué hace cada rueda:
  A) speed=+C, steer=0   -> las dos ruedas giran "parejo": signo de cada rueda y del contador
  B) speed=0,  steer=+C  -> las ruedas giran opuestas: cómo mezcla el firmware el giro
Con eso arma la matriz M que pasa de (speed, steer) del firmware a RPM "hacia adelante" de cada
rueda: [f_izq, f_der] = M @ [speed, steer]. El puente usa la inversa de M para pedir RPM.

"Adelante" no se puede medir con las ruedas en el aire: se indica con --speed-positivo-es
(atras = un speed positivo mueve al robot hacia atrás; así quedó en la interfaz de prueba).

Uso:  ros2 run hoverboard_bridge calibrar --port /dev/ttyUSB0 [--comando 80]
Guarda el resultado en ~/.ros/hoverboard_calibracion.yaml (lo lee el puente).
"""
import argparse
import os
import sys
import time

import numpy as np
import serial

from hoverboard_bridge.protocolo import FeedbackParser, delta_int16, make_command

SALIDA = os.path.expanduser('~/.ros/hoverboard_calibracion.yaml')


def prueba(ser, parser, speed, steer, t_rampa=0.7, t_medir=1.5):
    """Manda (speed, steer) y devuelve RPM promedio y avance de contadores en la ventana."""
    rpm_l, rpm_r, cnt0, cnt1 = [], [], None, None
    t0 = time.time()
    t_ini_medir = t0 + t_rampa
    t_fin = t_ini_medir + t_medir
    t_cmd = 0.0
    while time.time() < t_fin:
        now = time.time()
        if now >= t_cmd:
            ser.write(make_command(steer, speed))
            t_cmd = now + 0.05
        for fb in parser.feed(ser.read(256)):
            if now >= t_ini_medir:
                if cnt0 is None:
                    cnt0 = (fb.cnt_l, fb.cnt_r, now)
                cnt1 = (fb.cnt_l, fb.cnt_r, now)
                rpm_l.append(fb.rpm_l)
                rpm_r.append(fb.rpm_r)
        time.sleep(0.005)
    if not rpm_l or cnt0 is None:
        sys.exit('[calibrar] no llega feedback del hoverboard (¿puerto?, ¿encendido?)')
    d_l = delta_int16(cnt1[0], cnt0[0])
    d_r = delta_int16(cnt1[1], cnt0[1])
    return np.mean(rpm_l), np.mean(rpm_r), d_l, d_r, cnt1[2] - cnt0[2]


def parar(ser, parser, t=1.2):
    t_fin = time.time() + t
    while time.time() < t_fin:
        ser.write(make_command(0, 0))
        parser.feed(ser.read(256))
        time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser(description='Calibración del hoverboard (ruedas en el aire)')
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--comando', type=int, default=80, help='speed/steer de prueba (0..200)')
    ap.add_argument('--speed-positivo-es', choices=['adelante', 'atras'], default='atras')
    ap.add_argument('--si', action='store_true', help='no preguntar (ruedas ya en el aire)')
    ap.add_argument('--no-swap-wheels', action='store_true',
                    help='NO intercambiar ruedas (en Milo la "izquierda" del firmware es la derecha)')
    args = ap.parse_args()
    c = max(20, min(200, args.comando))
    sentido = 1.0 if args.speed_positivo_es == 'adelante' else -1.0

    if not args.si:
        r = input('¿Las ruedas están EN EL AIRE (sin tocar el suelo)? [s/N] ')
        if r.strip().lower() not in ('s', 'si', 'sí', 'y'):
            sys.exit('[calibrar] cancelado')

    ser = serial.Serial(args.port, 115200, timeout=0)
    parser = FeedbackParser()
    try:
        parar(ser, parser, 0.5)
        print(f'[calibrar] prueba A: speed={c} steer=0 ...')
        a = prueba(ser, parser, c, 0)
        parar(ser, parser)
        print(f'[calibrar] prueba B: speed=0 steer={c} ...')
        b = prueba(ser, parser, 0, c)
    finally:
        parar(ser, parser, 0.5)
        ser.close()

    print(f'  A: rpm_izq={a[0]:+.1f} rpm_der={a[1]:+.1f} cnt_izq={a[2]:+d} cnt_der={a[3]:+d} '
          f'({a[4]:.2f} s)')
    print(f'  B: rpm_izq={b[0]:+.1f} rpm_der={b[1]:+.1f} cnt_izq={b[2]:+d} cnt_der={b[3]:+d}')
    if min(abs(a[0]), abs(a[1])) < 5:
        sys.exit('[calibrar] en la prueba A alguna rueda casi no giró: revisar motores / comando')

    # Signo de cada rueda: k * rpm = RPM hacia adelante del robot.
    k_l = sentido * np.sign(a[0])
    k_r = sentido * np.sign(a[1])
    # Signo del contador y pasos por vuelta (rpm/60 * t = vueltas).
    c_l = sentido * np.sign(a[2]) if a[2] else k_l
    c_r = sentido * np.sign(a[3]) if a[3] else k_r
    tpr_l = abs(a[2]) / (abs(a[0]) / 60.0 * a[4]) if a[2] else 0.0
    tpr_r = abs(a[3]) / (abs(a[1]) / 60.0 * a[4]) if a[3] else 0.0

    m = np.array([[k_l * a[0], k_l * b[0]],
                  [k_r * a[1], k_r * b[1]]]) / c
    if abs(np.linalg.det(m)) < 1e-4:
        sys.exit('[calibrar] el giro (steer) no separa las ruedas: no se puede invertir la mezcla')
    # Giro positivo (steer>0): ¿qué rueda va hacia adelante?
    giro = 'izquierda' if m[1, 1] > m[0, 1] else 'derecha'
    print(f'  steer>0 gira el robot hacia la {giro}')
    print(f'  pasos de contador por vuelta: izq {tpr_l:.1f}  der {tpr_r:.1f} (esperado ~90)')

    os.makedirs(os.path.dirname(SALIDA), exist_ok=True)
    with open(SALIDA, 'w') as f:
        f.write('# Generado por: ros2 run hoverboard_bridge calibrar\n')
        f.write(f'# {time.strftime("%Y-%m-%d %H:%M")}  comando de prueba {c}, speed positivo = '
                f'{args.speed_positivo_es}\n')
        f.write(f'rpm_sign_left: {k_l:.0f}\nrpm_sign_right: {k_r:.0f}\n')
        f.write(f'tick_sign_left: {c_l:.0f}\ntick_sign_right: {c_r:.0f}\n')
        f.write(f'ticks_per_rev_measured: [{tpr_l:.1f}, {tpr_r:.1f}]\n')
        f.write('# [f_izq, f_der] (RPM hacia adelante) = mix @ [speed, steer]\n')
        f.write(f'mix: [{m[0,0]:.5f}, {m[0,1]:.5f}, {m[1,0]:.5f}, {m[1,1]:.5f}]\n')
        f.write('# true: la rueda "izquierda" del firmware es la derecha del robot (Milo, 10-11-2026)\n')
        f.write(f'swap_wheels: {"false" if args.no_swap_wheels else "true"}\n')
    print(f'[calibrar] guardado en {SALIDA}')


if __name__ == '__main__':
    main()
