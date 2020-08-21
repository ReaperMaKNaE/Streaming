# bluetooth : COM6
import serial

arduino = serial.Serial(
    port = 'COM6', baudrate=9600,
)

value1 = 101
value2 = 102
value3 = 203
value4 = 204
value5 = 100

while True:
    op = input()
    if op =='q':
        arduino.write(op.encode('utf-8'))
        break
    elif op == 'a':
        arduino.write(bytes(str(value1), "ascii"))
    elif op == 'b':
        arduino.write(bytes(str(value2), "ascii"))
    elif op == 'c':
        arduino.write(bytes(str(value3), "ascii"))
    elif op == 'd':
        arduino.write(bytes(str(value4), "ascii"))
    elif op == 'e':
        arduino.write(bytes(str(value5), "ascii"))
    else:
        arduino.write(op.encode('utf-8'))
