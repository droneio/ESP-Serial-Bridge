import serial

# Define the serial port and baud rate
serial_port = "/dev/ttyUSB0"  # Replace with the correct serial port for your system
baud_rate = 115200

# Open the serial port
try:
    ser = serial.Serial(serial_port, baud_rate)
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()


text_to_send = "Hello, Serial Port!"



while 1:

    # Send the text
    try:
        #
        #ser.write(text_to_send.encode())
        #print("Send")
        t = ser.read()
        print(f"Received: {t}")
    except Exception as e:
        print(f"Error sending data: {e}")

# Close the serial port
ser.close()