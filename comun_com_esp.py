import serial

# Open the virtual serial port for reading
ser = serial.Serial('/dev/pts/4', 9600)  # Replace with the correct virtual port

while True:
    if ser.in_waiting > 0:  # Check if there is data in the buffer
        line = ser.readline().decode('utf-8').rstrip()  # Read and decode the data
        print("Received:", line)
