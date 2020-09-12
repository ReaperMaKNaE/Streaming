#
# Copyright 2020 Joung Byung In, All Rights Reserved
#
# RegressionFilter.py
#
# SKKU
# School of Mechanical Engineering
# School of Electromagnetic Engineering
# Senior Student
#
# 2020.09.09 for ver 1.0
#
# NOTE :
#       This library can execute some mathmatical expression
#       such as calculating slope in array and expect next value
#       using regression analysis
#
#       ver 1.0 : predict next value for filtering
#

class RegressionAnalysis:
    def __init__(self):
        self.numData = 0
        self.dataArray = []

    def GetNumberOfData(self, numberOfData):
        self.numData = numberOfData

    def CalculateSlope(self, data1, data2):
        self.slope = (data1 - data2) / 50
        if self.slope > 10 :
            return 1
        else :
            return 0

    def SaveData(self, data):
        if len(self.dataArray) < self.numData:
            self.dataArray.append(data)
        else :
            self.dataArray.pop(0)
            self.dataArray.append(data)

    def CalculateFilteredData(self):
        numData = self.numData
        dataArray = self.dataArray

        avgX = numData * (numData + 1)/(2*numData)
        avgXsquare = numData * (numData + 1) * (2 * numData + 1) / (6 * numData)

        if len(dataArray) < numData :
            return dataArray[len(dataArray)-1]
        else:
            avgY = 0
            avgXY = 0

            for cnt in range(0, numData-1) :
                avgY += dataArray[cnt] / numData
                avgXY += (cnt + 1)*dataArray[cnt] /numData

            slope = (avgXY - avgX + avgY ) / (avgXsquare - avgX * avgX)
            tip = (avgY * avgXsquare - avgX * avgXY)/(avgXsquare - avgX * avgX)

            return slope * (numData + 1) + tip

