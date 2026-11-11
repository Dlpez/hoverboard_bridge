"""Protocolo serie del firmware hoverboard-firmware-hack-FOC (EFeru), sin ROS.

Comando (Jetson -> hoverboard), 8 bytes little-endian:
    start 0xABCD | steer int16 | speed int16 | checksum = start ^ steer ^ speed
Feedback (hoverboard -> Jetson), 22 bytes (versión actual del firmware, con contadores de rueda):
    start | cmd1 | cmd2 | speedR_meas | speedL_meas | wheelR_cnt | wheelL_cnt | batVoltage |
    boardTemp | cmdLed | checksum (XOR de todos los anteriores)
speedX_meas en RPM del motor; wheelX_cnt cuenta pasos de los sensores Hall (int16, da la vuelta);
batVoltage en centésimas de volt; boardTemp en décimas de °C.
"""
import struct

START_FRAME = 0xABCD
CMD_FMT = '<HhhH'
FB_FMT = '<HhhhhhhhhHH'
FB_SIZE = struct.calcsize(FB_FMT)  # 22
_START_BYTES = struct.pack('<H', START_FRAME)


def u16(x):
    return x & 0xFFFF


def make_command(steer, speed):
    """Arma el paquete de comando (steer y speed en -1000..1000)."""
    steer = int(max(-1000, min(1000, steer)))
    speed = int(max(-1000, min(1000, speed)))
    checksum = u16(START_FRAME ^ u16(steer) ^ u16(speed))
    return struct.pack(CMD_FMT, START_FRAME, steer, speed, checksum)


class Feedback:
    __slots__ = ('cmd1', 'cmd2', 'rpm_r', 'rpm_l', 'cnt_r', 'cnt_l', 'bat', 'temp', 'led')

    def __init__(self, vals):
        (_, self.cmd1, self.cmd2, self.rpm_r, self.rpm_l, self.cnt_r, self.cnt_l,
         bat, temp, self.led, _) = vals
        self.bat = bat / 100.0
        self.temp = temp / 10.0


class FeedbackParser:
    """Junta los bytes que llegan y devuelve los paquetes de feedback válidos."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(_START_BYTES)
            if i < 0:
                del self.buf[:-1]
                return out
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < FB_SIZE:
                return out
            vals = struct.unpack(FB_FMT, bytes(self.buf[:FB_SIZE]))
            chk = 0
            for v in vals[:-1]:
                chk ^= u16(v)
            if chk == vals[-1]:
                out.append(Feedback(vals))
                del self.buf[:FB_SIZE]
            else:
                self.bad += 1
                del self.buf[:2]


ENCODER_WRAP = 9000  # el firmware de Milo cuenta 0..8999 y vuelve a 0 (igual que el driver
                     # DataBot-Labs/hoverboard_ros2_control: ENCODER_MAX_VALUE 9000)


def delta_int16(new, old, wrap=ENCODER_WRAP):
    """Diferencia entre dos lecturas de un contador que da la vuelta cada `wrap` pasos.

    Se toma el salto más corto (entre -wrap/2 y +wrap/2): entre dos paquetes (~10 ms) la rueda
    avanza unos pocos pasos, así que 8998 -> 1 son +3 pasos, no -8997.
    """
    d = (new - old) % wrap
    return d - wrap if d >= wrap // 2 else d
