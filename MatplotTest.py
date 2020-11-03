from matplotlib import pyplot as plt
import numpy as np

PI = 3.141592

GoForward = 0
addNewObstacle = 0
frameNum = 0
originRPY = [0, 0, 0]
EulerAngle = [0, 0, 0]
AngleInBytes = [0, 0, 0]
initialized = 0

data = [0.1, -0.1, 0.2, -0.15, 0.11, 0.8, 0.9, 0.8, 0.5, 0.3]
x_plot=[]

for i in range(len(data)):
    x_plot.append(i+1)
if len(data) > 100:
    #새로운 데이터가 들어오면 pop 한개, 마지막에 데이터 append로 추가하는 방식으로 설정.
    plt.show()

plt.plot([1,2,3], [110,130,120])
plt.show()