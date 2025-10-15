from models.imu_recive import DM_Serial

ser = DM_Serial('/dev/ttyACM0', baudrate=921600, timeout=0.5)

while True:
    ser.read()
