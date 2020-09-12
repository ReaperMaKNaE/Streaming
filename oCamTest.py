import cv2

# oCamS-1CGN-U
# VideoCapture를 통해 얻어오는 값에는 IMU Sensor의 값도 있을 것
# 각 카메라는 8bit data bus를 통해 보냄
# USB Controller에서는 16bit data bus로 받음
#
capture = cv2.VideoCapture(1)

while True:
    ret, frame = capture.read()

    image = cv2.cvtColor(frame,cv2.COLOR_YUV2BGR_UYVY)

    img_b,img_g,img_r = cv2.split(image)

    cv2.imshow("VideoFrame",image)
    cv2.imshow("Blue", img_b)
    cv2.imshow("Green", img_g)
    cv2.imshow("Red", img_r)

    if cv2.waitKey(1) > 0 : break

capture.release()
cv2.destroyAllWindows()