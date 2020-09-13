import cv2
import numpy as np
import urllib.request
import serial

# To start real time streaming in raspberryPi
# mjpg_streamer -i "input_raspicam.so -vf" -o "output_http.so -p 8090
#                   -w /usr/local/share/mjpg_streamer/www/"
video = urllib.request.urlopen("http://192.168.0.18:8080/video_feed")
total_bytes = b''

#arduino = serial.Serial(
#    port = 'COM6', baudrate=9600,
#

fourcc = cv2.VideoWriter_fourcc(*'XVID')
writer = cv2.VideoWriter('output3.avi', fourcc, 25.0, (1280,480))

while(True):
    total_bytes += video.read(1024)
    b = total_bytes.find(b'\xff\xd9') # JPEG END
    if not b == -1:
        a = total_bytes.find(b'\xff\xd8') # JPEG start
        jpg = total_bytes[a:b+2] # actual image
        total_bytes = total_bytes[b+2:] # other informations

        #decode to colored image (another option is cv2.IMREAD_GRAYSCALE)
        img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        left = img[241:480, 1:640, :]
        right = img[241:480, 641:1280, :]

        #fgbg = cv2.createBackgroundSubtractorMOG2(varThreshold=100)

        #fgmask_left = fgbg.apply(left)
        #fgmask_right = fgbg.apply(right)

        #mixed = np.hstack([fgmask_left, fgmask_right])

        cv2.imshow('Dual_image',img) # display image while receving data
        #cv2.imshow('mixed', mixed)

        writer.write(img)
        #cv2.VideoWriter('example_mixed.avi', fourcc).write(mixed)
        if cv2.waitKey(1)==27:
            break

writer.release()
cv2.destroyAllWindows()