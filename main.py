#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
最小可跑示例：读取达妙 IMU 帧（非阻塞），Ctrl+C 立即关闭串口。
用法：
  python3 main_dm.py -p /dev/ttyACM0 -b 921600
"""
import argparse
import sys
import time

from models.dm_serial import DM_Serial

def parse_args():
    ap = argparse.ArgumentParser(description="DM IMU reader (non-blocking)")
    ap.add_argument("-p", "--port", default="/dev/ttyACM0", help="Serial port, e.g. /dev/ttyACM0 or COM3")
    ap.add_argument("-b", "--baud", type=int, default=921600, help="Baudrate, e.g. 921600/460800/115200")
    ap.add_argument("--sleep", type=float, default=0.001, help="Loop sleep in seconds to reduce CPU usage")
    return ap.parse_args()

def print_stats(dev: DM_Serial):
    total = dev.cnt_ok + dev.cnt_crc + dev.cnt_short + dev.cnt_nohdr
    ok_rate = (dev.cnt_ok / total) if total else 0.0
    print(f"[stats] ok={dev.cnt_ok} crc_fail={dev.cnt_crc} short={dev.cnt_short} nohdr={dev.cnt_nohdr} ok_rate={ok_rate:.3f}", flush=True)

def main():
    args = parse_args()
    dev = DM_Serial(args.port, args.baud)
    if not dev.is_open:
        print(f"[error] cannot open serial: port={args.port} baud={args.baud}", file=sys.stderr)
        sys.exit(1)

    t_stat = time.time()
    try:
        while True:
            pkt = dev.read()
            if pkt is not None:
                rid, (f1, f2, f3) = pkt
                # 常见：RID=0x03 对应 roll/pitch/yaw（按你之前的帧）
                if rid == 0x03:
                    print(f"RID=0x03  roll={f1:.3f}  pitch={f2:.3f}  yaw={f3:.3f}", flush=True)
                else:
                    print(f"RID=0x{rid:02X}  f1={f1:.3f}  f2={f2:.3f}  f3={f3:.3f}", flush=True)

            now = time.time()
            if now - t_stat >= 1.0:
                print_stats(dev)
                t_stat = now

            if args.sleep > 0.0:
                time.sleep(args.sleep)

    except KeyboardInterrupt:
        # Ctrl+C 立即销毁串口
        pass
    finally:
        dev.destory()
        print("[info] serial closed.", flush=True)

if __name__ == "__main__":
    main()
