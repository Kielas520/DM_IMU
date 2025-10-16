# dm_serial.py
# -*- coding: utf-8 -*-
"""
DM_Serial: 达妙 IMU 串口读取类（低延迟版）
- timeout 固定为 0（非阻塞）
- read(): 一次性读入串口缓冲里“所有可读字节”，解析出所有完整帧，只返回“最新一帧”，避免滞后
- destory(): 立即关闭串口（按你的拼写保留）；destroy() 为别名
- reopen(): 关闭并按相同 port/baud 重新打开
- CRC：默认“包含帧头 0x55,0xAA”计算；若失败会自动再尝试“不含帧头”

帧格式（与你之前日志一致）:
[0,1]=0x55,0xAA | [2]=? | [3]=RID | [4:16]=3*float32(LE) | [16:18]=CRC16(LE) | [18]=0x0A
"""
from __future__ import annotations

import struct
from typing import Optional, Tuple, List

import serial  # pip install pyserial

from .dm_crc import dm_crc16

HDR = b'\x55\xAA'
TAIL = 0x0A
FRAME_LEN = 19
VALID_RIDS = {0x01, 0x02, 0x03}

# 按你的帧实际情况：CRC 包含帧头，如有异构固件，自动兜底
SKIP_HDR_IN_CRC = False

class DM_Serial:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = 0.0  # 固定非阻塞
        self.ser: Optional[serial.Serial] = None
        self._buf = bytearray()

        # 统计
        self.cnt_ok = 0
        self.cnt_crc = 0
        self.cnt_short = 0
        self.cnt_nohdr = 0

        self._open()

    # ---------- 公共方法 ----------
    def read(self, max_bytes: int = 0) -> Optional[Tuple[int, Tuple[float, float, float]]]:
        """
        非阻塞地读取并解析串口数据。
        行为：一次性把串口里“当前可读”的全部字节拉进缓冲 -> 解析出所有完整帧 -> 返回最新一帧。
        这样能避免 backlog 导致的“看起来滞后”。
        :param max_bytes: 若>0则最多读取这么多字节；默认 0 表示不限，直接 in_waiting 全拉
        :return: (rid, (f1, f2, f3)) 或 None
        """
        if not self.ser or not self.ser.is_open:
            return None

        self._read_into_buf(max_bytes if max_bytes > 0 else None)
        frames = self._parse_all()
        return frames[-1] if frames else None

    def destory(self) -> None:
        """立即关闭串口（拼写保留）。"""
        if self.ser:
            try:
                self.ser.close()
            finally:
                self.ser = None

    # 别名
    def destroy(self) -> None:
        self.destory()

    def reopen(self) -> bool:
        """关闭并重新打开串口。"""
        self.destory()
        return self._open()

    @property
    def is_open(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    # ---------- 内部实现 ----------
    def _open(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout, write_timeout=0)
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            return True
        except Exception:
            self.ser = None
            return False

    def _read_into_buf(self, max_bytes: Optional[int]) -> int:
        """把当前串口里所有可读数据读入 _buf；返回读取字节数。"""
        n = getattr(self.ser, "in_waiting", 0) if self.ser else 0
        if max_bytes is not None and n > max_bytes:
            n = max_bytes
        if n <= 0:
            return 0
        self._buf.extend(self.ser.read(n))
        return n

    def _parse_all(self) -> List[Tuple[int, Tuple[float, float, float]]]:
        """从缓冲中尽可能解析出所有完整帧，返回列表。避免只取一帧造成积压。"""
        results: List[Tuple[int, Tuple[float, float, float]]] = []
        buf = self._buf
        start = 0

        while True:
            j = buf.find(HDR, start)
            if j < 0:
                # 只保留最后 1 个字节，避免帧头跨包
                keep = buf[-1:] if buf else b''
                self._buf = bytearray(keep)
                if buf:
                    self.cnt_nohdr += 1
                break

            if len(buf) - j < FRAME_LEN:
                # 不够一整帧，保留从帧头开始的尾部
                self._buf = bytearray(buf[j:])
                self.cnt_short += 1
                break

            frame = bytes(buf[j:j + FRAME_LEN])
            start = j + 1  # 快速向前移动

            # 尾字节
            if frame[-1] != TAIL:
                continue

            rid = frame[3]
            if rid not in VALID_RIDS:
                continue

            # 计算 CRC（默认含帧头，失败则尝试不含帧头的算法）
            if SKIP_HDR_IN_CRC:
                crc_calc = dm_crc16(frame[2:16])
            else:
                crc_calc = dm_crc16(frame[0:16])
            crc_wire = frame[16] | (frame[17] << 8)
            if crc_calc != crc_wire:
                alt = dm_crc16(frame[2:16]) if not SKIP_HDR_IN_CRC else dm_crc16(frame[0:16])
                if alt != crc_wire:
                    self.cnt_crc += 1
                    continue

            # 解 3 个 float32（LE）
            f1 = struct.unpack('<f', frame[4:8])[0]
            f2 = struct.unpack('<f', frame[8:12])[0]
            f3 = struct.unpack('<f', frame[12:16])[0]
            results.append((rid, (f1, f2, f3)))

            # 丢弃已消费的数据（到帧尾），并从头开始继续找
            buf = buf[j + FRAME_LEN:]
            start = 0

        # 若 buf 被我们裁剪过，则同步回 _buf
        if isinstance(buf, (bytes, bytearray)) and buf is not self._buf:
            self._buf = bytearray(buf)

        # 成功计数
        self.cnt_ok += len(results)
        return results
