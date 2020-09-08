#! /usr/bin/env python3

import liboCams
import cv2
import time
from flask import Flask, render_template, Response

devpath = liboCams.FindCamera('video0')

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

def gen():
  test = liboCams.oCams(devpath, verbose=0)

  test.Set(fmtlist[2])
  ctrllist = test.GetControlList()
  name =  test.GetName()
  test.Start()
   
  while True:

    frame = test.GetFrame(mode=1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BAYER_GB2BGR)
      
    rgb = cv2.resize(rgb, dsize=(1280,480), interpolation = cv2.INTER_AREA)
    ret, jpeg = cv2.imencode('.jpg', rgb)
    yield (b'--frame\r\n'
           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')
    #cv2.imshow('test', rgb)
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


  #test.Stop()  
  #cv2.destroyAllWindows()
  #test.Close()
  
app = Flask(__name__)

@app.route('/')
def index():
    #rendering webpage
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(gen(),mimetype='multipart/x-mixed-replace;boundary=frame')

if __name__ == '__main__':
    app.run(host='192.168.0.18', port='8080', debug=True)





