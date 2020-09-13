#
# Copyright 2020 Joung Byung In, All Rights Reserved
#
# DrawDepthMap.py
#
# SKKU
# School of Mechanical Engineering
# School of Electromagnetic Engineering
# Senior Student
#
# 2020.09.09 for ver 1.0
#
# NOTE :
#       Draw map from the video
#       based on Vision(OpenCV) and Contour
#
#       ver 1.0 : Detect obstacles and find how far they are
#                 Draw simple map with obstacles and mobile robot
#

import cv2
import numpy as np
import RegressionFilter as RF
import math
from matplotlib import pyplot as plt

# oCamS-1CGN-U
# White balance red component : 0x98090e
# White balance blue component : 0x98090f
# gain : 0x980913
# exposure : auto, 0x9a0901(absolute : 0x9a0902)

# Call image
video = cv2.VideoCapture("output2.avi")

# Save Images

# fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# originWriter = cv2.VideoWriter('origin.mp4', fourcc, 25.0, (1280,480))
# threshWriter = cv2.VideoWriter('thresh.mp4', fourcc, 25.0, (1280,480))
# contWriter = cv2.VideoWriter('cont.mp4', fourcc, 25.0, (1280,480))
# distanceWriter = cv2.VideoWriter('distance.mp4', fourcc, 25.0, (1280,480))

# Initialize values

count = 0
distanceRobotFromObstacle = []
addNewObstacleAtMap = 0
obstacle = [] # save location of obstacles
numObstacle = 100 # Assume Maximum number of obstacles are 100
robot = [320,480] # Save location of robot
frameNum = 0
obstacleUpdate = 0
robot_x_saved = 0
robot_y_saved = 0
updateDistance = 0

while(True):
    print('======================== Check Parameters =========================')
    print(' frameNum : ', frameNum)
    print(' count : ', count)
    print(' addNewObstacleAtMap : ', addNewObstacleAtMap)
    print(' numObstacle : ', numObstacle)
    print(' obstacleUpdate : ', obstacleUpdate)
    print(' updateDistance : ', updateDistance)
    print('======================= Parameters confirmed ======================')

    #initialize values
    robot_x = 0
    robot_y = 0
    map = np.zeros((480, 640, 3), np.uint8)

    # Take ret and frame. if video format is confirmed, ret will be 1.
    ret, frame = video.read()

    # As same as ret is 0
    if frame is None :
        break

    # Divide two sides
    left = frame[:,0:640,:]
    right = frame[:,640:1280,:]

    left_cont = left.copy()
    right_cont = right.copy()

    # Take Grayscale and Canny Edge to get key points
    left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

    # Threshold is 20, maximum binary value is 255.
    left_thresh_binary = cv2.threshold(left_gray, 20, 255, cv2.THRESH_BINARY)[1]
    right_thresh_binary = cv2.threshold(right_gray, 20, 255, cv2.THRESH_BINARY)[1]

    # remove boundary of image to prevent wrong detection
    left_thresh = 255 - left_thresh_binary
    right_thresh = 255 - right_thresh_binary

    # Get contours
    contours_left, hierarchy_left = cv2.findContours(left_thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours_right, hierarchy_right = cv2.findContours(right_thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    cx_l=[]
    cy_l=[]
    cx_r=[]
    cy_r=[]

    # Draw contrours
    for cnt_left in contours_left:
        cv2.drawContours(left_cont, [cnt_left], 0, (255,0,0), 3) # contour with blue color
        # if contour area is over than 8000, draw green circle at centroid
        if cv2.contourArea(cnt_left) > 9500:
            M_left = cv2.moments(cnt_left, False)
            cx_left = int(M_left['m10']/M_left['m00'])
            cy_left = int(M_left['m01']/M_left['m00'])

            #print values to calculate depth
            print('cx_left, cy_left : ', cx_left, 'and', cy_left)

            cx_l.append(cx_left)
            cy_l.append(cy_left)

            cv2.circle(left_cont, (cx_left,cy_left), 5, (0,255,0),-1)

    for cnt_right in contours_right:
        cv2.drawContours(right_cont, [cnt_right], 0, (255,0,0), 3) # contour with blue color
        # if contour area is over than 8000, draw green circle at centroid
        if cv2.contourArea(cnt_right) > 9500:
            M_right = cv2.moments(cnt_right, False)
            cx_right = int(M_right['m10'] / M_right['m00'])
            cy_right = int(M_right['m01'] / M_right['m00'])

            cx_r.append(cx_right)
            cy_r.append(cy_right)

            #print values to calculate depth
            print('cx_right, cy,right : ', cx_right, 'and', cy_right)

            cv2.circle(right_cont, (cx_right,cy_right), 5, (0,255,0),-1)

    # Save only contour except the distance value
    left_onlyCont = left_cont.copy()
    right_onlyCont = right_cont.copy()

    # Map parameter for obstacles
    cxlForMap = []
    cxrForMap = []
    distanceForMap = []

    # Check distance
    #
    # cxl and cxr is pixel number for center of the obstacle.
    # therefore, the obstacle number should be same as number of cxl in cx_l
    #                           Stereo Image
    #             -----------------------------------------
    #           |        LEFT        |        RIGHT        |
    #           |                    |                     |
    #           |-------->cx_l       |---------->cx_r      |
    #           |                    |                     |
    #           |                    |                     |
    #             ----------------------------------------
    # the unit is [pixel].

    # When the obstacle which we want to detect is observed,
    # take it and found distance and save the information of them
    # to the
    # distanceForMap (Save vertical distance value of obstacle)
    # cxlForMap (cx_l values)
    # cxrForMap (cx_r values)
    # Surely, we should consider the value of cyl and cyr
    # because of the rotation
    # But that would be treated as well after soon.

    # When the save is finished, mark the distance by using 'cv2.putText'
    # The first frame which got a obstacle saves the number of obstacle
    # When we found additional obstacle, set count as 0
    # lost obstacle means meaning nothing. just record the obstacle and
    # robot should avoid it.
    #
    #                       --- Found obstacle ---- Got other obstacle ---- lost obstacle ---
    # count                 :      0 -> 1                   0                     1
    # addNewObstacleAtMap   :      0 -> 1                   1                     1
    # obstacleUpdate        :      0 -> 1                 1->0                    1
    for cxl in cx_l:
        for cxr in cx_r:
            if abs(cxr-cxl) < 100 and abs(cxr-cxl) > 0:
                distance = int(abs(float(640*120/(cxr-cxl))))

                distanceForMap.append(distance)
                cxlForMap.append(cxl)
                cxrForMap.append(cxr)
                cv2.putText(left_cont, str(distance), (cxl,cy_l[cx_l.index(cxl)]),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                cv2.putText(right_cont, str(distance), (cxr,cy_r[cx_r.index(cxr)]),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

    print(' === Check distanceForMap before obstacle check === ')
    print('distanceForMap : ', distanceForMap)
    print('         === Finish check distanceForMap === ')

    # When obstacle is exist, update the value of distance
    print('==== Obstacle update will be proceeded. ====')
    if obstacle is not None and distanceForMap is not None:
        for obstacleParameter in obstacle:
            index = obstacle.index(obstacleParameter)
            obstacleDistance = obstacleParameter[2]

            for distance in distanceForMap:
                if abs(obstacleDistance - distance) < 200:
                    print('the distance is updated.')
                    obstacle[index][2] = distance
                    updateDistance = 1
                    break

            if updateDistance == 0 :
                if obstacleUpdate == 1:
                    break
                print('new obstacle is detected.')
                print('new obstacle will be added at next step')
                count = 0
                addNewObstacleAtMap = 1
                obstacleUpdate = 0
        print('==== check complete. ====')
    else :
        print('obstacle is None. Not update anything')
        print('========== Cancel update ============')

    # obstacle detection
    #
    # When lost obstacle, not record that. it would be saved to the map.
    #                           Stereo Image
    #             -----------------------------------------
    #           |        LEFT        |        RIGHT        |
    #             -----------------------------------------
    #           |          |         |           |         |
    #           |         \/         |          \/         |
    #           |     obstacle_y     |     obstacle_y      |
    #           |                    |                     |
    #           |                    |                     |
    #           |---->obstacle_x     |---->obstacle_x      |
    #             ----------------------------------------
    # distance means distance between obstacle and camera

    if count == 0:  # When first obstacle detected
        if addNewObstacleAtMap == 0:
            for distance in distanceForMap:
                if distance < 5000:
                    print('===== The first obstacle is detected ======')
                    index = distanceForMap.index(distance)
                    center = int((cxlForMap[index]+cxrForMap[index])/2)
                    obstacle_y = int(480-distance*480/5000)
                    obstacle.append([center, obstacle_y, distance])
                    count = 1
                    numObstacle = len(obstacle)

        else :      # When new obstacle detected, add new one.
            for distance in distanceForMap:
                if obstacleUpdate == 1:
                    break
                if distance < 5000:
                    print('===== New Obstacle is detected =====')
                    index = distanceForMap.index(distance)
                    print('detected obstacle is : ', index)
                    center = int((cxlForMap[index]+cxrForMap[index])/2)
                    obstacle_y = int(robot_y_saved-distance*480/5000)

                    # Check obstacles and add
                    for obstacleParameter in obstacle:
                        index=obstacle.index(obstacleParameter)
                        if abs(obstacleParameter[2]-distance) < 200 : # if the obstacle is origin
                            print('This obstacle[',index,'] is origin, not update.')
                            continue
                        else :  # if the obstacle is new one
                            print('=====new obstacle[', index,'] is detected.=====')
                            obstacle.append([center, obstacle_y, distance])
                            print('===== new obstacle[',index,'] is added. =====')
                            obstacleUpdate = 1

                    count = 1
                    numObstacle = len(obstacle)

    # Set obstacle Object for filtering.
    #for distance in distanceForMap:
    #     index = distanceForMap.index(distance)
    #     # First Frame
    #     if frame == 1 :
    #         obstacle1 = RF.RegressionAnalysis()
    #         obstacle1.GetNumberOfData(5)
    #     # Check the first obstacle is right
    #     for obstacleIndex in obstacle:
    #         indexObs = obstacle.index(obstacleIndex)
    #         if obstacleIndex[2] == distance :

    print(' Check obstacles ')
    print('number of Obstacles : ', numObstacle)
    print('obstacle : ', obstacle)
    print(' Draw Map ')

    # Draw robot's location - Localization

    for distance in distanceForMap:
        index = distanceForMap.index(distance)
        angle = int((cxlForMap[index]+cxrForMap[index])/2) - 320
        x_direction = angle
        y_direction = int(distance * 480 / 5000)

        for obstacleTitle in obstacle:
            if obstacleTitle[2] == distance :
                maxIndex = len(distanceForMap)
                index = obstacle.index(obstacleTitle)
                robot_x += (obstacle[index][0] - x_direction) / maxIndex
                robot_y += (obstacle[index][1] + y_direction) / maxIndex
                print('angle : ', angle)
                print('x_direction : ', x_direction)
                print('y_direction : ', y_direction)
            else :
                continue

    robot_x_saved = robot_x
    robot_y_saved = robot_y

    # Display the location of robot and obstacle
    print('robot x : ', robot_x_saved)
    print('robot y : ', robot_y_saved)
    cv2.circle(map, (int(robot_x_saved), int(robot_y_saved)), 10, (0,0,255), -1)
    for obstacleIndex in obstacle :
        index = obstacle.index(obstacleIndex)
        cv2.circle(map, (obstacle[index][0],obstacle[index][1]), 5, (0,255,0),-1)


    cv2.imshow('map', map)

    threshImage = np.hstack([left_thresh, right_thresh])
    contImage = np.hstack([left_onlyCont, right_onlyCont])
    distanceImage = np.hstack([left_cont, right_cont])

    # convert threshImage to save with RGB(not binary, binary : 8bits, RGB : 24bits)
    threshImage = cv2.cvtColor(threshImage, cv2.COLOR_GRAY2RGB)

    cv2.imshow('origin', frame)
    cv2.imshow('threshImage', threshImage)
    cv2.imshow('contImage', contImage)
    cv2.imshow('distanceImage', distanceImage)

    frameNum += 1

    # save video

    # originWriter.write(frame)
    # threshWriter.write(threshImage)
    # contWriter.write(contImage)
    # distanceWriter.write(distanceImage)

    if cv2.waitKey(1) == 27:
        break

video.release()

# save video

# originWriter.release()
# threshWriter.release()
# contWriter.release()
# distanceWriter.release()
cv2.waitKey(0)
cv2.destroyAllWindows()