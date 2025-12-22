import serial

port = "/dev/cu.usbserial-0001" # wrong not for blt
baud = 115200

ser = serial.Serial(port, baud, timeout=1)

while True:
  ser.write(b"ping\n")
  line = ser.readline().decode(errors="ignore").strip()
  if line:
    print("ESP32:", line)