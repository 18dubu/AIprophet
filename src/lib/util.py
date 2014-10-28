import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re,sys
from math import *

#load position data
def loadData(targetFile):
    location=[]
    nums = re.compile(r"[+-]?\d+(?:\.\d+)?")
    with open(targetFile,'r') as f:
        for line in f:
            line = line.rstrip()
            #print nums.finditer(str(line))
            a=[]
            for i in nums.finditer(str(line)):
                a.append(int(i.group(0)))
            location.append(a)
    return location

def animatePath(location):
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    import re
    def simPoints(simData):
        x, t = simData[0], maxYLim-simData[1]
        x_text.set_text(x_template%(x))
        y_text.set_text(y_template%(t))
        line.set_data(x, t)
        return line, x_text,y_text
    maxXLim = 854
    maxYLim = 480
    fig = plt.figure()
    ax = fig.add_subplot(111)
    line, = ax.plot([], [], 'bo', ms=5)
    ax.set_xlim(0, maxXLim)
    ax.set_ylim(0, maxYLim)
    x_template = 'X_value = %.1f '    # prints running simulation time
    x_text = ax.text(0.05, 0.95, '', transform=ax.transAxes)
    y_template = 'Y_value = %.1f '    # prints running simulation time
    y_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
    ani = animation.FuncAnimation(fig, simPoints, location, blit=False, interval=10, repeat=True)
    plt.show()
#sample usage
#animatePath(location)#input is a list of positions

#1D LIST INPUT
def calculateError1D(predictList,actualList):
    error = []
    if not len(predictList) == len(actualList):
        print 'input lists have different length'
        sys.exit()
    else:
        for i in range(len(predictList)):
            error.append(sqrt((float(predictList[i])-float(actualList[i]))**2))
        return error

#2D LIST WITHIN LIST INPUT
def calculateError2D(predictList,actualList):
    error = []
    if not len(predictList) == len(actualList):
        print 'input lists have different length'
        sys.exit()
    else:
        for i in range(len(predictList)):
            if len(predictList[i]) == len(actualList[i]):
                tmp = 0
                for j in range(len(predictList[i])):
                    tmp += (float(predictList[i][j])-float(actualList[i][j]))**2
                tmp = sqrt(tmp)
                error.append(tmp)
            else:
                print 'error input: cell length differs'
                sys.exit()
        return error


#assumption: the edges are formed by lines which are parallel with the video edges
#the smaller the neiPara is, the more strict the requirement is, which elimilate more outliers
def findEdge(X_pos,Y_pos,neiPara=5):
    x_min=None
    y_min=None
    x_max=None
    y_max=None
    X_pos = [x for x in X_pos if x != -1]
    Y_pos = [x for x in Y_pos if x != -1]
    X_pos = sorted(X_pos)
    Y_pos = sorted(Y_pos)
    flagx1 = 0
    flagx2 = 0
    for i in range(len(X_pos)):
        if flagx1>=1 and flagx2>=1:
            break
        if abs(X_pos[i]-X_pos[i+1]) < neiPara:
            x_min = X_pos[i]
            flagx1 += 1
        if abs(X_pos[::-1][i] - X_pos[::-1][i]) < neiPara:
            x_max = X_pos[::-1][i]
            flagx2 += 1
        else:
            i += 1
    flagy1=0
    flagy2=0
    for i in range(len(Y_pos)):
        if flagy1>=1 and flagy2>=1:
            break
        if abs(Y_pos[i]-Y_pos[i+1]) < neiPara:
            y_min = Y_pos[i]
            flagy1 += 1
        if abs(Y_pos[::-1][i] - Y_pos[::-1][i]) < neiPara:
            y_max = Y_pos[::-1][i]
            flagy2 += 1
        else:
            i += 1
    return x_min,x_max,y_min,y_max


def formatPrintList(list,eachSpace):
    for line in list:
        print '{:>8} {:>8} {:>8}'.format(*line)

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

