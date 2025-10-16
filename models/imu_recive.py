# -*- coding: utf-8 -*-
# 串口读取 + 帧同步 + CRC 校验（达妙 IMU USB 19B 帧）
# 需求：pip install pyserial
import serial
import struct
from .dm_crc import dm_crc16  # 你已有的表驱动 CRC：init=0xFFFF，覆盖 0..15 字节

class DM_Serial:
    def __init__(self, port: str, baud: int, timeout: float = 0.05):
        """
        :param port: 例如 '/dev/ttyACM0' 或 'COM3'
        :param baud: 115200 等
        :param timeout: 读超时，必须有，便于优雅退出
        """
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.ser.reset_input_buffer()
        self.buf = bytearray()

        # 帧格式（USB 模式，固定 19B）
        self.HDR = b'\x55\xAA'
        self.TAIL = 0x0A
        self.FRAME_LEN = 19
        self.VALID_RIDS = {0x01, 0x02, 0x03}  # ACC/GYRO/EULER

    def close(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    def _fill(self):
        """把当前可读的字节尽量读进缓冲（至少读1个，避免空转）"""
        n = self.ser.in_waiting
        self.buf.extend(self.ser.read(n if n else 1))

    def read(self):
        """
        读取并解析最多一帧。
        :return: (parsed, buf, why)
        parsed=None 表示未取到完整/有效帧；why ∈ {'nohdr','short','crc'}
        parsed=(rid,(f1,f2,f3)) 表示成功；why='ok'
        说明：
        - CRC 覆盖区间为 frame[0:16]（含 0x55 0xAA 到第三个 float 末尾），线上 CRC 小端在 [16],[17]。
        - 本方法内部维护 self.buf，调用者无需再写回。
        """
        # 先把新字节喂进缓冲
        self._fill()

        i = 0
        while True:
            j = self.buf.find(self.HDR, i)
            if j < 0:
                # 没有帧头：保留最后1字节防跨包
                self.buf = self.buf[-1:] if self.buf else bytearray()
                return None, self.buf, 'nohdr'

            # 不够一帧，保留尾巴等待下一批
            if len(self.buf) - j < self.FRAME_LEN:
                self.buf = bytearray(self.buf[j:])
                return None, self.buf, 'short'

            frame = bytes(self.buf[j:j+self.FRAME_LEN])
            i = j + 1  # 失败时从下一个字节继续扫描

            # 尾字节检查
            if frame[-1] != self.TAIL:
                continue

            rid = frame[3]
            if rid not in self.VALID_RIDS:
                continue

            # ---- CRC：覆盖 0..15 字节；线上 CRC 小端（L 在前）----
            crc_wire = frame[16] | (frame[17] << 8)
            if dm_crc16(frame[0:16]) != crc_wire:
                # 轻量反同步：推进一字节继续；这里返回给上层统计一下
                self.buf = bytearray(self.buf[i:])
                return None, self.buf, 'crc'

            # 解析三个 float32 小端
            f1 = struct.unpack('<f', frame[4:8])[0]
            f2 = struct.unpack('<f', frame[8:12])[0]
            f3 = struct.unpack('<f', frame[12:16])[0]

            # 成功：消费这一帧
            self.buf = bytearray(self.buf[j+self.FRAME_LEN:])
            return (rid, (f1, f2, f3)), self.buf, 'ok'
