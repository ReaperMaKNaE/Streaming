import cv2
import numpy as np
import serial

# To start real time streaming in raspberryPi
# mjpg_streamer -i "input_raspicam.so -vf" -o "output_http.so -p 8090
#                   -w /usr/local/share/mjpg_streamer/www/"
cap = cv2.VideoCapture("http://192.168.5.101:8090/?action=stream")

arduino = serial.Serial(
    port = 'COM6', baudrate=9600,
)

while(cap.isOpened()):
    ret, frame = cap.read()
    origin = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150, apertureSize = 3)
    lines = cv2.HoughLines(edges, 1, np.pi/180, 150)
    comps = 0
    if lines is not None:
        for line in lines:
            rho, theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            comps = comps + 1
            cv2.line(frame, (x1,y1), (x2,y2), (0,0,255), 2)

    if comps == 0:
        arduino.write(b'a')
    elif comps > 0 and comps <= 4:
        arduino.write(b'b')
    elif comps > 4 and comps <= 8:
        arduino.write(b'c')
    elif comps > 8 and comps <= 12:
        arduino.write(b'd')
    elif comps > 12 and comps <= 16:
        arduino.write(b'e')
    else:
        arduino.write(b'k')

    if ret:
        cv2.imshow('video',frame)
        cv2.imshow('origin',origin)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    else:
        break

cap.release()
cv2.destroyAllWindows()