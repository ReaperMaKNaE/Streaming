from matplotlib import pyplot as plt
import cv2
import numpy as np
import urllib.request
#import math
#import time
#import serial
import warnings
warnings.simplefilter("ignore", DeprecationWarning)

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

GoForward = 0
addNewObstacle = 0
frameNum = 0
originRPY = [0, 0, 0]
EulerAngle = [0, 0, 0]
AngleInBytes = [0, 0, 0]
initialized = 0
numData = 0

angleVelX = []
angleVelY = []
angleVelZ = []
angleAccX = []
angleAccY = []
angleAccZ = []
x_plot = []

AngleInBytesArr_roll = []
AngleInBytesArr_pitch = []

#arduino = serial.Serial(
#    port = 'COM4', baudrate=38400,
#)

plt.ion()
while(True):
    accelValuePass = 1
    gyroValuePass = 1
    total_bytes += video.read(1024)
    b = total_bytes.find(b'\xff\xd9') # JPEG END

    if not b == -1:
        numData += 1
        a = total_bytes.find(b'\xff\xd8') # JPEG start
        jpg = total_bytes[a:b+2] # actual image
        total_bytes = total_bytes[b+2:] # other informations

        #decode to colored image (another option is cv2.IMREAD_GRAYSCALE)
        img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        #left = img[241:480, 1:640, :]
        #right = img[241:480, 641:1280, :]

        angle_start = total_bytes.find(b'\x42\x4D\xF6\x04\x00\x00')
        aX = int.from_bytes(total_bytes[angle_start+6:angle_start+9], "big")
        aY = int.from_bytes(total_bytes[angle_start+9:angle_start+12], "big")
        aZ = int.from_bytes(total_bytes[angle_start+12:angle_start+15], "big")
        gX = int.from_bytes(total_bytes[angle_start+15:angle_start+18], "big")
        gY = int.from_bytes(total_bytes[angle_start+18:angle_start+21], "big")
        gZ = int.from_bytes(total_bytes[angle_start+21:angle_start+24], "big")
        roll = int.from_bytes(total_bytes[angle_start+24:angle_start+26], "big")-512
        pitch = int.from_bytes(total_bytes[angle_start+26:angle_start+28], "big")-512
        yaw = int.from_bytes(total_bytes[angle_start+28:angle_start+30], "big")-512

        accel = [aX/10000-10, aY/10000-10, aZ/10000-10]
        gyro = [gX/10000-10, gY/10000-10, gZ/10000-10]

        for accelValue in accel:
            if abs(accelValue) > 5:
                accelValuePass = 0

        for gyroValue in gyro:
            if abs(gyroValue) > 5:
                gyroValuePass = 0

        if (accelValuePass + gyroValuePass) == 2:
            if numData > 100 :
                angleVelX.pop(0)
                angleVelX.append(gyro[0])

                angleVelY.pop(0)
                angleVelY.append(gyro[1])

                angleVelZ.pop(0)
                angleVelZ.append(gyro[2])

                angleAccX.pop(0)
                angleAccX.append(accel[0])

                angleAccY.pop(0)
                angleAccY.append(accel[1])

                angleAccZ.pop(0)
                angleAccZ.append(accel[2])
            else :
                x_plot.append(numData)
                angleVelX.append(gyro[0])
                angleVelY.append(gyro[1])
                angleVelZ.append(gyro[2])
                angleAccX.append(accel[0])
                angleAccY.append(accel[1])
                angleAccZ.append(accel[2])

        cv2.imshow('img', img)
        if cv2.waitKey(1)==27:
            break

        if abs(roll) < 180 :
            if abs(pitch) < 180 :
                if abs(yaw) < 180 :
                    if numData > 100:
                        AngleInBytesArr_roll.pop(0)
                        AngleInBytesArr_roll.append(roll)

                        AngleInBytesArr_pitch.pop(0)
                        AngleInBytesArr_pitch.append(pitch)
                    else:
                        AngleInBytesArr_roll.append(roll)
                        AngleInBytesArr_pitch.append(pitch)
                    AngleInBytes = [roll, pitch, yaw]

            if initialized == 0:
                print('initializing... wait...')
                #print('accel, gyro, AngleInBytes : ', accel, gyro, AngleInBytes)
                print('Angle In Bytes : ', AngleInBytes)
                plt.clf()
                plt.subplot(221)
                plt.plot(x_plot, angleVelX, x_plot, angleVelY, x_plot, angleVelZ)
                plt.title('Gyro Value(Omega(rad/s))')
                plt.subplot(222)
                plt.plot(x_plot, angleAccX, x_plot, angleAccY, x_plot, angleAccZ)
                plt.title('Accel Value(m/(10*s^2))')
                plt.subplot(223)
                plt.plot(AngleInBytesArr_roll)
                plt.xlabel('Roll')
                plt.subplot(224)
                plt.plot(AngleInBytesArr_pitch)
                plt.xlabel('Pitch')
                originRPY = AngleInBytes
                plt.pause(0.001)
                #if numData > 90:
                    #arduino.write(bytes(str(int(roll)+90), "ascii"))
                    #print('senddata : ', int(roll)+90)
                    #time.sleep(0.3)

            else:
                if cv2.waitKey(1)==27:
                    #arduino.write(bytes(str(0), "ascii"))
                    break
        plt.show()

cv2.destroyAllWindows()