import matplotlib
import cv2
import numpy as np
import urllib.request
import math
import time
import serial

#
# 체크할 것 :
#           그냥 전진 시 각속도, 가속도, 롤, 피치.
#           컨트롤 적용 후 각속도, 가속도, 롤, 피치.
#
#           그냥 회전 시 각속도, 가속도, 롤, 피치
#           컨트롤 적용 후 각속도, 가속도, 롤, 피치.
#
#           각속도/가속도 같은 matplotlib.
#           롤/피치 같은 matplotlib.
#
#           이후 옆에 영상 하나 슥 첨부하고.
#
#           컨트롤 넣은 걸로 비누 배달하는 영상 찍고 끝내기.
#

PI = 3.141592

video = urllib.request.urlopen("http://192.168.137.193:8080/video_feed")
total_bytes = b''

obstacle = [] # save location of obstacles and distance from the robot in each frame
numObstacle = 100 # Assume Maximum number of obstacles are 100
robot = [320,480] # Save location of robot
addNewObstacle = 0
obstacleUpdate = []
frameNum = 0
velocity = 0
robot_x_saved = 0
robot_y_saved = 0
previous_robot_x = 0
previous_robot_y = 0
count = 0
addNewObstacleAtMap = 0
initialized = 0
originRPY = [0, 0, 0]
EulerAngle = [0, 0, 0]
GoForward = 0
GoDistance = 0
CheckNonControl = 0
AngleInBytes = [0, 0, 0]
map_x_size, map_y_size = 800, 500

arduino = serial.Serial(
    port = 'COM4', baudrate=9600,
)

while(True):
    if GoForward == 0 :
        print(' Enter How many do robot go forward in [meter] (maximum - 5m) : ')
        print(' If you wanna end this, type 255. ')
        GoDistance = int(input())
        if GoDistance == 255 :
            break
        print(' If you wanna move robot, type 210. For check non-control, type 230 ')
        op = int(input())
        if op == 210:
            arduino.write(bytes(str(210), "ascii"))
            GoForward = 1
        elif op == 230:
            arduino.write(bytes(str(210), "ascii"))
            CheckNonControl = 1

    elif CheckNonControl == 1:
        print(' Check non - control. ')
        time.sleep(5)
        CheckNonControl = 0
        arduino.write(bytes(str(0), "ascii"))
        print(' Finish Check non - control. ')

    else :
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

            angle_start = total_bytes.find(b'\x42\x4D\xF6\x04\x00\x00')
            roll = int.from_bytes(total_bytes[angle_start+6:angle_start+8], "big")
            pitch = int.from_bytes(total_bytes[angle_start+8:angle_start+10], "big")
            yaw = int.from_bytes(total_bytes[angle_start+10:angle_start+12], "big")

            if abs(roll) < 180 :
                if abs(pitch) < 180 :
                    if abs(yaw) < 180 :
                        AngleInBytes = [roll-512, pitch-512, yaw-512]

            if AngleInBytes[0] != 0  and initialized == 0:
                print('initializing... wait...')
                print('AngleInBytes : ', AngleInBytes)
                originRPY = AngleInBytes
                time.sleep(1)
            else:
                if cv2.waitKey(1)==27:
                    arduino.write(bytes(str(0), "ascii"))
                    break

cv2.destroyAllWindows()