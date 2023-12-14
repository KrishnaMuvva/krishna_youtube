import serial
import random

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

val = random.randint(0,9)

#arduino.write(str.encode(str(val)))

#arduino.write(str(val).encode('utf-8'))

#arduino.write(bytes(str(val).encode()))

xyz = "hello"

arduino.write(bytes(xyz, 'utf-8'))

#arduino.write(b'test')

#arduino.readline()

print(val)

#print(arduino.readline())