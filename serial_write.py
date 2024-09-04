import serial
import time

# Open the serial port for writing (Replace '/dev/ttyUSB0' with your port)
ser = serial.Serial('/dev/pts/3', 9600)
time.sleep(2)  # Wait for the serial connection to initialize

while True:
    ser.write(b'Hello from writer!\n')  # Send data
    time.sleep(1)  # Wait for 1 second before sending the next message
