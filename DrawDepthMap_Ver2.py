#
# Copyright 2020 Joung Byung In, All Rights Reserved
#
# DrawDepthMap_Ver2.py
#
# SKKU
# School of Mechanical Engineering
# School of Electromagnetic Engineering
# Senior Student
#
# 2020.10.22 for ver 2.0
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
import math

PI = 3.141592

# Call image
video = cv2.VideoCapture("output2.avi")

# Initialize values
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
eulerAngle = [0, 0, 0]
count = 0
addNewObstacleAtMap = 0

while(True):
    print('======================== Check Parameters =========================')
    print(' frameNum : ', frameNum)
    print(' obstacle : ', obstacle)
    print(' robot : ', robot)
    print(' addNewObstacle : ', addNewObstacle)
    print(' numObstacle : ', numObstacle)
    print(' obstacleUpdate : ', obstacleUpdate)
    print(' velocity : ', velocity)
    print(' eulerAngle : ', eulerAngle)
    print(' count : ', count)
    print(' addNewObstacleAtMap : ', addNewObstacleAtMap)
    print('======================= Parameters confirmed ======================')

    #initialize values
    robot_x = 0
    robot_y = 0
    map = np.zeros((500, 800, 3), np.uint8)

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
    left_thresh_binary = cv2.threshold(left_gray, 19, 255, cv2.THRESH_BINARY)[1]
    right_thresh_binary = cv2.threshold(right_gray,19, 255, cv2.THRESH_BINARY)[1]

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

    # Draw contours
    for cnt_left in contours_left:
        cv2.drawContours(left_cont, [cnt_left], 0, (255,0,0), 3) # contour with blue color
        # if contour area is over than 8000, draw green circle at centroid
        if cv2.contourArea(cnt_left) > 6000:
            M_left = cv2.moments(cnt_left, False)
            cx_left = int(M_left['m10']/M_left['m00'])
            cy_left = int(M_left['m01']/M_left['m00'])
            if cy_left > 400 :
                continue

            #print values to calculate depth
            print('cx_left, cy_left : ', cx_left, 'and', cy_left)

            cx_l.append(cx_left)
            cy_l.append(cy_left)

            cv2.circle(left_cont, (cx_left,cy_left), 5, (0,255,0),-1)

    for cnt_right in contours_right:
        cv2.drawContours(right_cont, [cnt_right], 0, (255,0,0), 3) # contour with blue color
        # if contour area is over than 8000, draw green circle at centroid
        if cv2.contourArea(cnt_right) > 6000:
            M_right = cv2.moments(cnt_right, False)
            cx_right = int(M_right['m10'] / M_right['m00'])
            cy_right = int(M_right['m01'] / M_right['m00'])
            if cy_right > 400 :
                continue

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
            if cxr < 550 and cxl < 550:
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

    # before update, we should check which one will be updated or not.
    # let me call it as poplist.
    popList = []
    if distanceForMap is not None:
        for distance in distanceForMap:
            index = distanceForMap.index(distance)
            popList.append([cx_l[index], cx_r[index], distanceForMap[index]])

    # At this time, the distanceForMap is ...
    #
    # obstacle = [cx1, cy1, distance1_Not_Updated], [cx2, cy2, distance2_Not_Updated], ...
    # distanceForMap = [distance1, distance2, ... ]
    # Therefore to make them as one, we shoudl consider them as index.
    #
    # Therefore, the meaning of the values could be
    #
    # "There are obstacles in the map.(or one)
    #  The detected one has cx{%d}.format(i) and cy{%d}.format(i) location in the map
    #  and the distance from the robot is distance[i]
    #
    # From this values, you can find the locations of robot and obstacles.

    # When obstacle is exist, update the value of distance.
    #
    # To update the value, we should check the velocity and the rotate angle of robot.
    # Let assume the rotate angle of robot is
    # eulerAngle = [roll, pitch, yaw]
    #
    # rotateAngle means how the robot rotates based on map
    # tileAngle means how the inside parts of the robot tilt
    #

    velocity = 0        # Get velocity value from the raspberryPI


    tiltAngle = 0
    rollAngle = 0
    eulerAngle = [0,0, 0]

    print('==== Obstacle update will be proceeded. ====')
    if obstacle is not None and distanceForMap is not None:
        # Obstacles are detected in this frame.
        for obstacleParameter in obstacle:
            index = obstacle.index(obstacleParameter)
            obstacleDistance = obstacleParameter[2]

            for distance in distanceForMap:
                # check distance and location of obstacle
                # if location of obstacle is similar to before, update.
                if abs(obstacleDistance - distance) < 250:
                    if (tiltAngle - int(obstacleParameter[0]/320*100)) < 10:
                        obstacle[index][2] = distance
                        obstacleUpdate[index] = 1

                        #
                        # before pop, check the distance relationship
                        # between pixel map and distance map
                        #
                        '''
                        xDiscrepancy = abs(robot_x_saved - obstacle[index][0])
                        yDiscrepancy = abs(robot_y_saved - obstacle[index][1])
                        xDiscrepancySqua = xDiscrepancy * xDiscrepancy
                        yDisCrepancySqua = yDiscrepancy * yDiscrepancy
                        print('xDiscrepancy and yDiscrepancy : ', xDiscrepancy, yDiscrepancy)
                        print('square value of each discrepancy : ',
                              xDiscrepancySqua, yDisCrepancySqua)
                        print('sum of them : ', xDiscrepancySqua + yDisCrepancySqua)

                        distanceInPixelMap = int(math.sqrt(xDiscrepancySqua + yDisCrepancySqua))
                        print('distance : ', distance, ' distanceInPixelMap : ', distanceInPixelMap)
                        '''
                        # You can find the distance relationship between two is
                        # 2560/262 = (distance / distacneInPixelMap)

                        # pop which is updated.
                        for distanceValue in popList:
                            if distanceValue[2] == distance:
                                popListIndex = popList.index(distanceValue)
                                popList.pop(popListIndex)
                        print('the distance is updated.')

        # Update Obstacles which are not detected
        countIndexForObstacleUpdate = 0
        for updateDataOfObstacle in obstacleUpdate:
            if updateDataOfObstacle == 0:
                # Check the distance relationship between pixel map and distance map
                xDiscrepancy = abs(robot_x_saved - obstacle[countIndexForObstacleUpdate][0])
                yDiscrepancy = abs(robot_y_saved - obstacle[countIndexForObstacleUpdate][1])
                xDiscrepancySqua = xDiscrepancy * xDiscrepancy
                yDiscrepancySqua = yDiscrepancy * yDiscrepancy
                distance = int(math.sqrt(xDiscrepancySqua+yDiscrepancySqua)*2560/262)
                obstacle[countIndexForObstacleUpdate][2] = distance
                obstacleUpdate[countIndexForObstacleUpdate] = 1
                #check data values
                '''
                print('xDiscrepancy , yDiscrepancy : ', xDiscrepancy, yDiscrepancy)
                print('xDiscrepancySqua , yDiscrepancySqua : ', xDiscrepancySqua, yDiscrepancySqua)
                print('sum of them : ', xDiscrepancySqua + yDiscrepancySqua)
                print('square root of sum : ', math.sqrt(xDiscrepancySqua + yDiscrepancySqua))
                print('distance : ', distance)
                '''
            countIndexForObstacleUpdate += 1

        # Check new obstacles
        # The addition of obstacles is next step.
        # This is just for check update new obstacle.
        # First, check all value of obstacle is updated.
        # popList is not empty, then there should be new obstacles.
        if popList is None:
            print('new obstacle is detected.')
            print('popList(the new one) is : ', popList)
            print('new obstacle will be added at next step')
            count = 0
            addNewObstacleAtMap = 1
        print('==== check complete. ====')
    else :
        print('obstacle is None. Not update anything')
        print('========== Cancel update ============')

    print(' Check obstacles before add new one')
    print('number of Obstacles : ', numObstacle)
    print('obstacle : ', obstacle)
    print(' Draw Map ')

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
            for obstacleParameter in popList:
                if obstacleParameter[2] < 5000:
                    print('===== New Obstacle is detected =====')
                    index = popList.index(obstacleParameter)
                    print('detected new obstacle is : ', obstacleParameter)
                    # center is value of (cx_l+cx_r)/2 in integer type
                    # obstacle_y means y location of obstacle
                    center = int((obstacleParameter[0] + obstacleParameter[1]) / 2)
                    obstacle_y = int(robot_y_saved - obstacleParameter[2] * 480 / 5000)
                    print('=====new obstacle[', obstacleParameter, '] is detected.=====')
                    obstacle.append([center, obstacle_y, obstacleParameter[2]])
                    print('===== new obstacle[', obstacleParameter, '] is added. =====')

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

    # append obstacles to draw Map

    if len(distanceForMap) != 0:
        obstacleListForMap = []

        for obstacleParameter in obstacle:
            for distance in distanceForMap:
                if obstacleParameter[2] == distance:
                    obstacleListForMap.append(obstacleParameter)

        # Draw robot's location - Localization
        for distance in distanceForMap:
            index = distanceForMap.index(distance)
            angle = int((cxlForMap[index]+cxrForMap[index])/2) - 320
            x_direction = angle
            y_direction = int(distance * 480 / 5000)

            for obstacleTitle in obstacleListForMap:
                if obstacleTitle[2] == distance :
                    maxIndex = len(distanceForMap)
                    index = obstacleListForMap.index(obstacleTitle)
                    robot_x += (obstacleListForMap[index][0] - x_direction) / maxIndex
                    robot_y += (obstacleListForMap[index][1] + y_direction) / maxIndex
                    print('angle : ', angle)
                    print('x_direction : ', x_direction)
                    print('y_direction : ', y_direction)
                else :
                    continue

        robot_x_saved = robot_x
        robot_y_saved = robot_y
        previous_robot_x = robot_x_saved
        previous_robot_y = robot_y_saved
    else:
        robot_x_saved = previous_robot_x
        robot_y_saved = previous_robot_y
        robot_x_saved += int(math.sin(tiltAngle*PI/180)*velocity*262/2560)
        robot_y_saved -= int(math.cos(tiltAngle*PI/180)*velocity*262/2560)
        previous_robot_x = robot_x_saved
        previous_robot_y = robot_y_saved

    # Display the location of robot and obstacle
    print('robot x : ', robot_x_saved)
    print('robot y : ', robot_y_saved)
    cv2.circle(map, (int(robot_x_saved), int(robot_y_saved)), 10, (0,0,255), -1)
    for obstacleIndex in obstacle :
        index = obstacle.index(obstacleIndex)
        cv2.circle(map, (obstacle[index][0],obstacle[index][1]), 5, (0,255,0),-1)

    obstacleUpdate = np.zeros(shape=(len(obstacle),), dtype=np.int8)

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

    # if you wanna no stop here, change it cv2.waitKey(1)==27
    if cv2.waitKey(0)==27 :
        break

print('frameNum : ', frameNum)

video.release()

# save video

# originWriter.release()
# threshWriter.release()
# contWriter.release()
# distanceWriter.release()

cv2.waitKey(0)
cv2.destroyAllWindows()