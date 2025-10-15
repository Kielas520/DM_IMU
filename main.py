from models.imu_recive import DM_Serial

def main():
    ser = None
    try:
        ser = DM_Serial('/dev/ttyACM0', baudrate=921600, timeout=0.5)
        while True:
            ser.read()
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected, closing serial...")
    finally:
        if ser is not None and ser.ser.is_open:
            ser.ser.close()
            print("[INFO] Serial port closed safely.")

if __name__ == "__main__":
    main()
