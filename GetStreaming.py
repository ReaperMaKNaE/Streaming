import cv2
import numpy as np

cap = cv2.VideoCapture("http://192.168.5.101:8090/?action=stream")

while(cap.isOpened()):
    ret, frame = cap.read()
    origin = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150, apertureSize = 3)
    lines = cv2.HoughLines(edges, 1, np.pi/180, 150)

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

            cv2.line(frame, (x1,y1), (x2,y2), (0,0,255), 1)

    if ret:
        cv2.imshow('video',frame)
        cv2.imshow('origin',origin)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    else:
        break

cap.release()
cv2.destroyAllWindows()