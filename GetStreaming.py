import cv2
import numpy as np
import urllib.request
import serial

# To start real time streaming in raspberryPi
# mjpg_streamer -i "input_raspicam.so -vf" -o "output_http.so -p 8090
#                   -w /usr/local/share/mjpg_streamer/www/"
video = urllib.request.urlopen("http://192.168.137.193:8080/video_feed")
total_bytes = b''

#bytes_angles = urllib.request.urlopen("http://192.168.137.193:8080/angles")


# arduino = serial.Serial(
#    port = 'COM6', baudrate=9600,
#

fourcc = cv2.VideoWriter_fourcc(*'XVID')
writer = cv2.VideoWriter('S-heroCheck.avi', fourcc, 25.0, (1280,480))

while(True):
    total_bytes += video.read(1024)
    b = total_bytes.find(b'\xff\xd9') # JPEG END
    #totalAngleBytes += bytes_angles.read(1024)

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
        angle_start = total_bytes.find(b'\x42\x4D\xF6\x04\x00\x00')
        roll = int.from_bytes(total_bytes[angle_start+6:angle_start+8], "big")
        pitch = int.from_bytes(total_bytes[angle_start+8:angle_start+10], "big")
        yaw = int.from_bytes(total_bytes[angle_start+10:angle_start+12], "big")

        AngleInBytes = [roll-512, pitch-512, yaw-512]
        print('AngleInBytes : ', AngleInBytes)

        writer.write(img)
        #cv2.VideoWriter('example_mixed.avi', fourcc).write(mixed)

        if cv2.waitKey(1)==27:
            break

writer.release()
cv2.destroyAllWindows()