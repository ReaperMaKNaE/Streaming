# bluetooth : COM6
import serial
import time

arduino = serial.Serial(
    port = 'COM4', baudrate=9600,
)

value1 = 101
value2 = 102
value3 = 203
value4 = 204
value5 = 100
i = 0

while True:
    if i < 255:
        i += 10
    else :
        i = 0

    #arduino.write(bytes(str(i), "ascii"))
    #time.sleep(1)

    op = int(input())
    if op == 100:
        arduino.write(bytes(str(0), "ascii"))
        break
    else:
        arduino.write(bytes(str(op), "ascii"))
    '''
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
    '''