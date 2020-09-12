#! /usr/bin/env python3

import liboCams
import cv2
import time

devpath = liboCams.FindCamera('oCam-1CGN-U')

if devpath is None:
  print ('oCam Device Not Found!')
  exit()

test = liboCams.oCams(devpath, verbose=0)

fmtlist = test.GetFormatList()
for fmt in fmtlist:
  print (fmt)

ctrlist = test.GetControlList()
for key in ctrlist:
  print (key, hex(ctrlist[key]))

test.SetControl(ctrlist[b'Gain'], 60)
test.SetControl(ctrlist[b'Exposure (Absolute)'],200)
test.Close()

if True:
  test = liboCams.oCams(devpath, verbose=0)

  test.Set(fmtlist[2])
  ctrllist = test.GetControlList()
  name =  test.GetName()
  test.Start()
   
  while True:

    frame = test.GetFrame()
    rgb = cv2.cvtColor(frame, cv2.COLOR_BAYER_GB2BGR)
      
    cv2.imshow('test', rgb)
    char = cv2.waitKey(1)
    if char == 27:
      break
    else:
      if char == ord('i'):
        val = test.GetControl(ctrlist[b'Gain'])
        test.SetControl(ctrlist[b'Gain'],val+1)
      elif char == ord('k'):
        val = test.GetControl(ctrlist['Gain'])
        test.SetControl(ctrlist[b'Gain'],val-1)
      elif char == ord('l'):
        val = test.GetControl(ctrlist[b'Exposure (Absolute)'])
        test.SetControl(ctrlist[b'Exposure (Absolute)'],val+1)
      elif char == ord('j'):
        val = test.GetControl(ctrlist[b'Exposure (Absolute)'])
        test.SetControl(ctrlist[b'Exposure (Absolute)'],val-1)


  test.Stop()  
  cv2.destroyAllWindows()
  test.Close()





