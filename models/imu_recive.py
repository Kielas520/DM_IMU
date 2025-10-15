import serial
import struct

class DM_Serial():
    def __init__(self, port: str, baudrate: int, timeout: float):
        self.ser = serial.Serial(port, baudrate, timeout = timeout)

    def crc16(data: bytes):
        # 这里需要用说明书附录的CRC16程序，先写个占位
        return 0

    def read(self):
        data = self.ser.read(19)  # 读取一帧
        if len(data) == 19 and data[0] == 0x55 and data[1] == 0xAA and data[3] == 0x03:
            # 提取 roll pitch yaw (4字节 float, little-endian)
            roll = struct.unpack('<f', data[4:8])[0]
            pitch = struct.unpack('<f', data[8:12])[0]
            yaw = struct.unpack('<f', data[12:16])[0]

            print(f"Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°")

