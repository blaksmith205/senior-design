import serial
import time

arduino_serial = serial.Serial('COM7', 10 ** 6)
sensor_serial = serial.Serial('COM4', 10 ** 6)

time.sleep(5)
while True:
    try:
        sensor_data = sensor_serial.readline()
        arduino_serial.write(sensor_data)
        arduino_response = arduino_serial.readline()
        print(arduino_response.decode("utf-8"))
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        break

sensor_serial.close()
arduino_serial.close()