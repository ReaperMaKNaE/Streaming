import cv2
import numpy as np
import time
from myCamCap import *

def my1CGN_Viwer():
    key = 0
    cap = myCamCapture()
    if cap.GetConnectedCamNumber() == 0:
        print("oCam not Found...")
    else:
        #print (cap.CamGetDeviceInfo(0))
        cap.CamOpen( DevNo = 0, Resolution = (480,640), FramePerSec = 45.0, BytePerPixel = 1)

        start_time = time.time()
        cap.CamStart()
        #time.sleep(0.1)
        cap.CamSetCtrl(myCamCapture.CTRL_EXPOSURE, -5)
        count = 0
        fail_Cnt = 0
        while(True):
            ret, frame = cap.CamGetImage()
            if ret is False:
                continue
            count += 1
            
            color = cv2.cvtColor(frame, cv2.COLOR_BAYER_GB2BGR)
            cv2.imshow('oCam-1CGN-U', color)
            
            key = cv2.waitKey(1)
            if key == 27:
                break
            
        end_time = time.time()
        
        #print('FPS= ', count/(end_time-start_time))
        cv2.destroyAllWindows()
        cap.CamStop()
        cap.CamClose()
        
        return key


my1CGN_Viwer()
