# bluetooth : COM6
import serial

arduino = serial.Serial(
    port = 'COM6', baudrate=9600,
)

while True:
    print("insert op : ", end='')
    op = input()
    if op =='q':
        arduino.write(op.encode())
        break
    elif op == 'a':
        arduino.write(b'a')
    elif op == 'b':
        arduino.write(b'b')
    elif op == 'c':
        arduino.write(b'c')
    elif op == 'd':
        arduino.write(b'd')
    elif op == 'e':
        arduino.write(b'e')
    else:
        arduino.write(b'k')
