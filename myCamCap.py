#python 3.6
#Preinstall opencv, numpy

#define INFO_USB_TYPE		1
#define INFO_SERIAL_NUM		2
#define INFO_MODEL_NAME		3
#define INFO_DATE_TIME		4

#define CTRL_BRIGHTNESS		1
#define CTRL_CONTRAST		2
#define CTRL_HUE		3
#define CTRL_SATURATION		4
#define CTRL_EXPOSURE		5

#typedef void* CAMPTR;
#
#/**
# *  CamOpen
# *      Open the connected oCam. When the function is failure, than return NULL.
# *
# *      - Input:
# *          const int ID        : Camera number (e.g. 0, 1, 2 ...)
# *          const int Width     : Width of the output image (e.g. 640)
# *          const int Height    : Height of the output image (e.g. 480)
# *          const double Fps    : Frame per second (e.g. 30.0)
# *          void(*FtnCB)(void*) : Pointer of the callback function
# *          void* Para          : Parameter of the callback function
# *
# *      - Output:
# *          CAMPTR              : Pointer(Handle) of the opened camera
# */
#LIBCAMCAP_API CAMPTR CamOpen(const int ID, const int Width, const int Height, const double FPS, void(*FtnCB)(void *Para, void *Data), void *Para);
#
#/**
# *  CamStart
# *      Start the video streaming.
# *      
# *      - Input:
# *          CAMPTR ptrCam       : Handle of the opened camera
# *
# *      - Output:
# *          int                 : Failed(0) or Success(1)
# */
#LIBCAMCAP_API int CamStart(CAMPTR ptrCam);
#
#/**
# *  CamStop
# *      Stop the video streaming.
# *      
# *      - Input:
# *          CAMPTR ptrCam       : Handle of the opened camera
# *
# *      - Output:
# *          int                 : Failed(0) or Success(1)
# */
#LIBCAMCAP_API int CamStop(CAMPTR ptrCam);
#
#/**
# *  CamClose
# *      Close the opened camera; Release the camera handle.
# *      
# *      - Input:
# *          CAMPTR ptrCam       : Handle of the opened camera
# */
#LIBCAMCAP_API void CamClose(CAMPTR& ptrCam);
#
#/**
# *  CamGetImage
# *      Get camera image
# *      
# *      - Input:
# *          CAMPTR ptrCam       : Handle of the opened camera
# *          BYTE* pImage        : Pointer of image
# *
# *      - Output:
# *          int                 : Failed(0) or Success(1)
# */
#LIBCAMCAP_API int CamGetImage(CAMPTR ptrCam, BYTE* pImage);
#
#/**
# *  CamSetCtrl
# *      Set camera control parameter
# *      
# *      - Input:
# *          CAMPTR ptrCam       : Handle of the opened camera
# *          int CtrlPara        : Control parameter
# *          int Value           : Control value
# *
# *      - Output:
# *          int                 : Failed(0) or Success(1)
# */
#LIBCAMCAP_API int CamSetCtrl(CAMPTR ptrCam, int CtrlPara, long Value);
#
#/**
# *  CamGetCtrl
# *      Get camera control parameter
# *      
# *      - Input:
# *          CAMPTR ptrCam       : Handle of the opened camera
# *          int CtrlPara        : Control parameter
# *          int* Value          : Control value
# *
# *      - Output:
# *          int                 : Failed(0) or Success(1)
# */
#LIBCAMCAP_API int CamGetCtrl(CAMPTR ptrCam, int CtrlPara, long* Value);
#
#/**
# *  CamGetCtrlRange
# *      Get camera parameter ranges
# *      
# *      - Input:
# *          CAMPTR ptrCam       : Handle of the opened camera
# *          int CtrlPara        : Control parameter
# *          int* MinValue       : Control min value
# *          int* MaxValue       : Control max value
# *
# *      - Output:
# *          int                 : Failed(0) or Success(1)
# */
#LIBCAMCAP_API int CamGetCtrlRange(CAMPTR ptrCam, int CtrlPara, long* MinValue, long* MaxValue);
#
#/**
# *  GetConnectedCamNumber
# *      Search the connected oCam-5CRO-U and return the number of the connected camera.
# *      
# *      - Output:
# *          int                 : Failed(-1, 0) or the number of connected camera
# */
#LIBCAMCAP_API int GetConnectedCamNumber();
#
#/**
# *  CamGetDeviceInfo
# *      Get device information.
# *      
# *      - Input:
# *          CAMPTR ptrCam       : Handle of the opened camera
# *          int    Info         : Type of information
# *
# *      - Output:
# *          char*               : Information string of Camera
# */
#LIBCAMCAP_API char* CamGetDeviceInfo(const int ID, int Info);
#
import platform
import ctypes
import numpy as np

class myCamCapture():
    CTRL_BRIGHTNESS	= ctypes.c_int(1)
    CTRL_CONTRAST	= ctypes.c_int(2)
    CTRL_HUE		= ctypes.c_int(3)
    CTRL_SATURATION	= ctypes.c_int(4)
    CTRL_EXPOSURE       = ctypes.c_int(5)
    CTRL_GAIN           = ctypes.c_int(6)
    CTRL_WB_BLUE        = ctypes.c_int(7)
    CTRL_WB_RED         = ctypes.c_int(8)
    def __init__(self):
        try:
            if platform.architecture()[0] == '64bit':
                self.mydll = ctypes.cdll.LoadLibrary("./libCamCap-x64.dll")
                #self.mydll = ctypes.CDLL(".\\libCamCap-amd64.dll")
            else:
                self.mydll = ctypes.cdll.LoadLibrary(".\\libCamCap-x86.dll")
            self.mydll.CamGetDeviceInfo.restype = ctypes.c_char_p
            self.mydll.CamOpen.restype = ctypes.POINTER(ctypes.c_int)
        except WindowsError as Error:
            print (Error)
            raise Exception('libCamCap-amd64.dll or libCamCap-x86.dll not found')
            
        self.cam = None
        self.resolution = (0,0)
            

    def GetConnectedCamNumber(self):
        return int(self.mydll.GetConnectedCamNumber())

    def CamGetDeviceInfo(self, devno):
        info = dict()
        for idx, param in enumerate(('USB_Port', 'SerialNo', 'oCamName', 'FWVersion')):
            info[param] = self.mydll.CamGetDeviceInfo(int(devno), idx+1)
        return info
    
    def CamGetDeviceList(self):
        CamCount = self.GetConnectedCamNumber()
        DeviceList = list()
        for idx in range(CamCount):
            dev = self.CamGetDeviceInfo(idx)
            dev['devno'] = idx
            DeviceList.append(dev)
        return DeviceList
            
    def CamStart(self):
        if self.cam == None: return 
        ret = self.mydll.CamStart(self.cam)

    def CamGetImage(self):
        if self.cam == None: return 0, None
        ret = self.mydll.CamGetImage(self.cam,  ctypes.c_char_p(self.bayer.ctypes.data))
        if ret == 1:
            return True, self.bayer
        else:
            return False, None  

    def CamStop(self):
        if self.cam == None: return
        ret = self.mydll.CamStop(self.cam)

    def CamClose(self):
        if self.cam == None: return
        ret = self.mydll.CamClose(ctypes.byref(self.cam))
        self.cam = None

    def CamGetCtrl(self, ctrl):
        if self.cam == None: return
        val = ctypes.c_int()
        ret = self.mydll.CamGetCtrl(self.cam, ctrl, ctypes.byref(val))
        return val.value
    
    def CamSetCtrl(self, ctrl, value):
        if self.cam == None: return
        val = ctypes.c_int()
        val.value = value
        ret = self.mydll.CamSetCtrl(self.cam, ctrl, val)

    def CamGetCtrlRange(self, ctrl):
        if self.cam == None: return
        val_min = ctypes.c_int()
        val_max = ctypes.c_int()
        ret = self.mydll.CamGetCtrlRange(self.cam, ctrl, ctypes.byref(val_min), ctypes.byref(val_max))            
        return val_min.value, val_max.value

    def CamOpen(self, **options):
        DevNo = options.get('DevNo')
        FramePerSec = options.get('FramePerSec')
        Resolution = options.get('Resolution')
        BytePerPixel = options.get('BytePerPixel')

        try:
            devno = DevNo
            (h, w) = Resolution
            pixelsize = BytePerPixel
            fps = FramePerSec
            self.resolution = (w, h)
            self.cam = self.mydll.CamOpen(ctypes.c_int(devno), ctypes.c_int(w), ctypes.c_int(h), ctypes.c_double(fps), 0, 0)
            self.bayer = np.zeros((h,w,pixelsize), dtype=np.uint8)
            return True
        except WindowsError:
            return False
        
CTRL_PARAM = {
    'Brightness':myCamCapture.CTRL_BRIGHTNESS,
    'Contrast':myCamCapture.CTRL_CONTRAST,
    'Hue':myCamCapture.CTRL_HUE,
    'Saturation':myCamCapture.CTRL_SATURATION,
    'Exposure':myCamCapture.CTRL_EXPOSURE,
    'Gain':myCamCapture.CTRL_GAIN,
    'WB Blue':myCamCapture.CTRL_WB_BLUE,
    'WB Red':myCamCapture.CTRL_WB_RED
}

def adjust_gamma(image, gamma=1.0):
	# build a lookup table mapping the pixel values [0, 255] to
	# their adjusted gamma values
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
 
	# apply gamma correction using the lookup table
	return cv2.LUT(image, table)
        
#----------------------------------------------------------------------
if __name__ == '__main__':
    import cv2
    import numpy as np
    import time
    
    cap = myCamCapture()
    if cap.GetConnectedCamNumber() == 0:
        print("oCam not Found...")
    else:
        print (cap.CamGetDeviceInfo(0))
        #print (cap.CamGetDeviceList())
        cap.CamOpen( DevNo = 0, Resolution = (480,640), FramePerSec = 30.0, BytePerPixel = 1)
        
        start_time = time.time()
        cap.CamStart()

        cap.CamSetCtrl(myCamCapture.CTRL_EXPOSURE, -11)
        count = 0
        while(True):
            ret, frame = cap.CamGetImage()

            count += 1
            color = cv2.cvtColor(frame, cv2.COLOR_BAYER_GB2BGR)
            cv2.imshow('oCam-1CGN-U', color)
            
            key = cv2.waitKey(1)
            if key == 27:
                break

        end_time = time.time()
        
        print('FPS= ', count/(end_time-start_time))
        cv2.destroyAllWindows()
        cap.CamStop()
        cap.CamClose()
        
