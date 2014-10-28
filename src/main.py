############import section############
import re, sys,copy
import os.path
#basic functions in animate path, find edges, calculate errors

#import lib.util as util
from lib.util import *

# from udacity matrix.py
from lib.matrix import *

#functions in implementing Kalman filter and other necessary functions
from lib.filter import *
######################################


class point:
    def __init__(self, x=0, y=0, speed=0, speedChange=0, heading=0, turnAngle=0, hitEdge=0):
        self.x = x
        self.y = y
        self.speed = speed
        self.speedChange = speedChange
        self.heading = heading
        self.turnAngle = turnAngle
        self.hitEdge = hitEdge


def checkHit(x, y):
    hitEdge = 0
    if x >= xEdgeUpper:
        hitEdge += 100
    if x <= xEdgeLower:
        hitEdge += 1000
    if y >= yEdgeUpper:
        hitEdge += 1
    if y <= yEdgeLower:
        hitEdge += 10
    return hitEdge


def pointList2ListInList(pointList):  #[X_pos,Y_pos,speed,speedChange,heading,turnAngle,hitEdge]
    fieldNum = 7
    mainTable = [[0 for i in range(len(pointList))] for j in range(fieldNum)]
    for i in range(len(pointList)):
        mainTable[0][i] = pointList[i].x
        mainTable[1][i] = pointList[i].y
        mainTable[2][i] = pointList[i].speed
        mainTable[3][i] = pointList[i].speedChange
        mainTable[4][i] = pointList[i].heading
        mainTable[5][i] = pointList[i].turnAngle
        mainTable[6][i] = pointList[i].hitEdge
    return mainTable

#mainTable = [X_pos,Y_pos,speed,speedChange,heading,turnAngle,hitEdge]

def fitAngle(rawAngle):
    if rawAngle % (2.0 * pi) > pi:
        return rawAngle % (2.0 * pi) - 2 * pi
    else:
        return rawAngle % (2.0 * pi)


def ifCanTurnRight(point):
    start = 3.0 / 8.0 * pi
    end = 3.0 / 4.0 * pi
    referenceDir = pi / 2.0
    test = point.hitEdge
    match = {1: 0.0, 10: pi, 100: pi / 2.0, 1000: -pi / 2.0}

    if test != 0:
        for i in sorted(match.keys()):
            if test % (i*10) != 0:
                test -= test % (i*10)
                extremHardDir = float(match[i])
                if point.heading > fitAngle(start + extremHardDir - referenceDir) and point.heading < fitAngle(end + extremHardDir - referenceDir):
                    return False
    return True

def positionAllowed(x,y,xmin,xmax,ymin,ymax):
    if x< xmin or x>xmax or y<ymin or y>ymax:
        return False
    else:
        return True

def ifCanEndure(point):
    sightStep = 2.0
    furtureX = point.x + sightStep * point.speed*cos(point.heading)
    furtureY = point.y + sightStep * point.speed*sin(point.heading)
    if positionAllowed(furtureX,furtureY,xmin,xmax,ymin,ymax):
        return True
    else:
        return False

def turnRight(heading,angle):
    return fitAngle(heading - angle)
def turnLeft(heading,angle):
    return fitAngle(heading + angle)


def getMatrixSimilarity(listInList1, listInList2):
    if len(listInList1) == len(listInList2) and len(listInList1[0]) == len(listInList2[0]):
        print


def suggestNext(pointPath, allPoints):
    neiPointNum = 5  # see the number of neighbour points

    currentPoint = pointPath[-1]
    if currentPoint.hitEdge > 0:  # the point should hit, only refer to those previous points that hit
        print
    else:  # the point is not hit, refer to points that do not hit
        targetPoints = pointPath[-neiPointNum:]
        targetList = pointList2ListInList(targetPoints)
        simScore = [0 for z in range(len(allPoints))]
        for j in range(neiPointNum, (len(allPoints) - 1)):
            if allPoints[j].hitEdge == 0:
                candidatePoints = allPoints[j - neiPointNum:j]
                candidateList = pointList2ListInList(candidatePoints)
                simScore[j] = getMatrixSimilarity(targetList, candidateList)
            else:
                simScore[j] = 0
            #use the following point information of the most similar points as prediction


#robot position prediction
#main function:
#regression Factor: trend that gets robot back to strait line, if 1 no regression, if factor: >0 and <1, tend to factorize turning angle
def predict(endFrame=0, predictLength=63, regressionFactor=1.0, actually=1, quiet=True):
    if not actually:
        if endFrame <= predictLength:
            print 'endFrame too early to get validation data.'
            return False
    if regressionFactor < 0.0 or regressionFactor > 1.0:
        print "Warning: regression factor should be within [0,1]"
        return False
    if actually != 0 and actually != 1:
        print "Warning: actually parameter should be either 0 (selecting end frame, for training) or 1 (actually prediction of 63 frames after video ends)"
        return False

    train = None
    predictSteps = None
    validation = None
    if actually == 0:
        end = endFrame  #290
        predictSteps = predictLength  #63
        train = allPoints[:end - predictSteps]  # element should be point class
        validation = allPoints[
                     end - predictSteps:end]  #allPoints[80:100]  # element should be point class,should have  the same length with predictions
    #true prediction

    if actually == 1:
        predictSteps = 63
        train = allPoints
        validation = []

    #generate prediction robot
    predictBot = robot(train[-1].x, train[-1].y, train[-1].heading, train[-1].turnAngle,
                       averSpeed)  # para: x, y, heading direction, turning angle, speed
    predictBot.set_noise(0.0, 0.0, 0.0)
    target_bot = predictBot

    prediction = []
    predictedPath = train[:-2]  # not included the last point, will be append in following
    previousSpeed = train[-1].speed
    hitPrevious = 0
    for i in range(predictSteps):
        sensed = target_bot.sense_all()  # x,y,speed,heading,turning

        speedChanged = sensed[2] - previousSpeed
        previousSpeed = sensed[2]

        hitOrNot = checkHit(sensed[0], sensed[1])
        #print hitOrNot
        #print target_bot.heading
        sensedPoint = point(sensed[0], sensed[1], sensed[2], speedChanged, sensed[3], sensed[4],
                            hitOrNot)  #x=0,y=0,speed=0,speedChange=0,heading=0,turnAngle=0,hitEdge=0

        prediction.append([int(sensedPoint.x), int(sensedPoint.y)])
        predictedPath.append(sensedPoint)

        #parameter modification process
        ###################################################################
        #hit edges, rule based
        ###
        speedUpFactor = 2.0 #BIGGER: SLOWER IN ACCELATING
        speedHitDownFactor = 1.0  # FRACTION OF SPEED DROP WHEN HIT
        headingHitTurningFactor = 60/180.0*pi  #TURING ANGLE WHEN HIT, PREFERENCE OF RIGHT TURN
        ###
        if int(sensedPoint.hitEdge) == 10 or int(sensedPoint.hitEdge) == 1:  # hit y edges
            #allow lingering for several steps
            if hitPrevious ==0:
                target_bot.distance += (averSpeed-target_bot.distance)/speedUpFactor

            if not ifCanEndure(sensedPoint):

                if abs(target_bot.heading)<60/180.0*pi and abs(target_bot.heading)>120/180.0*pi:
                    target_bot.heading = -1.0 * target_bot.heading #reflect
                else:
                    if ifCanTurnRight(sensedPoint):
                        target_bot.distance = speedHitDownFactor * target_bot.distance
                        target_bot.heading  = turnLeft(target_bot.heading, headingHitTurningFactor)
                    else:
                        target_bot.distance = speedHitDownFactor * target_bot.distance
                        target_bot.heading  = turnRight(target_bot.heading,headingHitTurningFactor)
            hitPrevious += 1
        if int(sensedPoint.hitEdge) >= 100 or int(sensedPoint.hitEdge) == 1000:
            if hitPrevious ==0:
                target_bot.distance += (averSpeed-target_bot.distance)/speedUpFactor
            if not ifCanEndure(sensedPoint):

                if abs(target_bot.heading)<150/180.0*pi and abs(target_bot.heading)>30/180.0*pi:
                    if sensedPoint.heading >= 0:
                        target_bot.heading = abs(abs(abs(target_bot.heading) - pi) % pi)
                    else:
                        target_bot.heading = -1.0 * abs(abs(abs(target_bot.heading) - pi) % pi)
                else:
                    if ifCanTurnRight(sensedPoint):
                        target_bot.distance = speedHitDownFactor * target_bot.distance
                        target_bot.heading  = turnLeft(target_bot.heading,headingHitTurningFactor)
                    else:
                        target_bot.distance = speedHitDownFactor * target_bot.distance
                        target_bot.heading  = turnRight(target_bot.heading,headingHitTurningFactor)
            hitPrevious += 1
        #preference
        if int(sensedPoint.hitEdge) == 0:
            hitPrevious = 0
            target_bot.heading -= headingPreference
            target_bot.distance += (averSpeed-target_bot.distance)/speedUpFactor
        #target_bot.turning = turningPreference

        if int(sensedPoint.hitEdge) > 0:
            target_bot.heading -= headingPreferenceHit
            if hitPrevious>10:
                tmpPoint = copy.deepcopy(sensedPoint)
                tmpPoint.heading = turnRight(tmpPoint.heading,90/180.0*pi)
                if ifCanEndure(tmpPoint):
                    target_bot.heading = turnRight(sensedPoint.heading,40/180.0*pi)
                else:
                    target_bot.heading = turnLeft(sensedPoint.heading,40/180.0*pi)

        #decrease turning if necessary, in case circle
        target_bot.turning = regressionFactor * target_bot.turning

        #change turning if stuck in a place
        ####################################################################
        target_bot.move_in_circle()

    #use validation set to calculate error
    if not quiet:
        print '##########################'
        print 'PREDICTION: '
        print prediction
    if len(validation) > 0:  #validation
        true = []
        if len(validation) == len(prediction):
            for i in range(len(validation)):
                true.append([validation[i].x, validation[i].y])
            errorList = calculateError2D(prediction, true)
            averError = sum(errorList) / len(errorList)
            errorList = ['%.5f' % elem for elem in errorList]
            if not quiet:
                print 'VALIDATION: '
                print true
                print 'ERROR: '
                print errorList
                print
                print "Average error rate: ", averError
                print '##########################'
            return prediction, float(averError), true,

        else:
            print 'warning: validation set length differs with prediction length'
            return False
    if actually == 1:
        return prediction



###############################################################################################################
#start of the process
#change the location if needed
#file format is the same as training_video1-centroid_data/ testing_video-centroid_data
targetFile = './data/testing_video-centroid_data'  #'../data/training_video1-centroid_data'#'../data/testing_video-centroid_data'
outputFile = open('./result/testing_video-centroid_data_prediction','w')

if not os.path.isfile(targetFile):
    print 'Warning: can not find input file: ', targetFile
    sys.exit()

###Read position data from file with format: list
location = loadData(targetFile)

#x,y position in format of list
X_pos = [x[0] for x in location if x[0] != -1 and x[1] != -1]
Y_pos = [x[1] for x in location if x[0] != -1 and x[1] != -1]


#find the possible edge positions
#exclude the outliers with a certain parameter
xmin, xmax, ymin, ymax = findEdge(X_pos, Y_pos)
#print xmin,xmax,ymin,ymax
###animate path given position list
#animatePath(location)

nearEdgeTole = 0.005  #tole of egde range: 0.05*(max_xrange-min-xrange), regard as edge
xEdgeUpper = xmax - nearEdgeTole * (xmax - xmin)
xEdgeLower = xmin + nearEdgeTole * (xmax - xmin)
yEdgeUpper = ymax - nearEdgeTole * (ymax - ymin)
yEdgeLower = ymin + nearEdgeTole * (ymax - ymin)

print '########Detected Boundary########'
print 'X_upper: >', xEdgeUpper
print 'X_low: <', xEdgeLower
print 'Y_upper: >', yEdgeUpper
print 'Y_lower: <', yEdgeLower


#rule based algorithm in tagging the possibility of a position being hit/not hit (the edges)
#
hitEdge = [0 for i in range(len(X_pos))]
turnAngle = [0 for i in range(len(X_pos))]
speedChange = [0 for i in range(len(X_pos))]
speed = [0 for i in range(len(X_pos))]
heading = [0 for i in range(len(X_pos))]
for i in range(len(X_pos)):
    turntmp = getTurn(i, X_pos, Y_pos)
    if turntmp:
        turnAngle[i] = turntmp
    speedChangetmp = getSpeedChange(i, X_pos, Y_pos)
    if speedChangetmp:
        speedChange[i] = speedChangetmp
    speedtmp = getSpeed(i, X_pos, Y_pos)
    if speedtmp:
        speed[i] = speedtmp

    diretmp = getDirec(i, X_pos, Y_pos)
    if diretmp:
        heading[i] = diretmp
    if turntmp >= (pi / 6) or speedChangetmp > 1:
        hitEdge[i] = checkHit(X_pos[i], Y_pos[i])
        if 0:
            print i, X_pos[i], Y_pos[i], getDirec(i, X_pos, Y_pos), getDirec(i + 1, X_pos, Y_pos)
            if getDirec(i, X_pos, Y_pos) is False:
                print getDirec(i - 1, X_pos, Y_pos)


#calculate robot preference when hit or not hit
##########################
notHitTurning = []
notHitHeading = []
hitTurning = []
hitHeading = []
hitTurningDic = {}
hitHeadingDic = {}
for i in range(len(turnAngle)):
    if hitEdge[i] == 0:
        notHitTurning.append(turnAngle[i])
        notHitHeading.append(heading[i])
    if hitEdge[i] > 0:
        preHitHeading = None
        preHitTurning = None
        postHitTurning = None
        postHitHeading = None
        for j in range(i):
            pre = i - j
            if hitEdge[pre] == 0:
                preHitHeading = heading[pre]
                preHitTurning = turnAngle[pre]
                break
        for k in range(1, len(turnAngle) - i):
            post = i + k
            if hitEdge[post] == 0:
                postHitHeading = heading[post]
                postHitTurning = turnAngle[post]
                break

        if preHitTurning is not None and postHitTurning is not None and postHitHeading is not None and preHitHeading is not None:
            hitHeadingDic[preHitHeading] = postHitHeading
            hitTurningDic[preHitTurning] = postHitTurning
            hitTurning.append(postHitTurning - preHitTurning)
            hitHeading.append(postHitHeading - preHitHeading)
turningPreference = sum(notHitTurning) / len(notHitTurning)
headingPreference = sum(notHitHeading) / len(notHitHeading)
turningPreferenceHit = sum(hitTurning) / len(hitTurning)
headingPreferenceHit = sum(hitHeading) / len(hitHeading)
averSpeed = sum(speed) / len(speed)

print"#########DETAIL OF REBOT#########"
print 'Average Speed of the robot: ',averSpeed
print "Turning Preference: ", turningPreference
print "Heading Preference: ", headingPreference
print "Heading Preference When Hit Boundary: ", headingPreferenceHit
print "Turning Preference When Hit Boundary: ", turningPreferenceHit
#print "hitHeadingDic: ", hitHeadingDic
#print "hitTurningDic: ", hitTurningDic
print '########Cross Validation#########'
allPoints = []
for i in range(len(X_pos)):
    allPoints.append(point(X_pos[i], Y_pos[i], speed[i], speedChange[i], heading[i], turnAngle[i], hitEdge[i]))

#if __name__ == "__main__":
###################################
#bulk testing code, can test the overall error for one video and thus select parameters
#
singleInvest = 0 # if not 0, test a single prediction, input the stop frame for the test
realPrediction= 0 # if not 0 ( can only be 0/1), will predict the following 63 frames after the video ends

##if singleInvest and realPrediction are both 0, test all possible predictions every 20 frames and calculate the overall
##error rate
if not singleInvest and not realPrediction:
    drawEvery = 0
    findOutlierCondition = 0
    upLimit = 100
    predictionLength = 63  #63
    errorList = []
    outlierPositionDic = {}
    n = 0
    for testPoint in range(predictionLength + 1, len(X_pos)-predictionLength, 20):
        n += 1
        prediction, error, true = predict(testPoint, predictionLength, 0.6, 0, True)
        if error:
            errorList.append(error)
        if error > upLimit:
            outlierPositionDic[error] = testPoint
            if findOutlierCondition:
                print 'Drawing True Path: '
                animatePath(true)
                print "Drawing Predicted Path: "
                animatePath(prediction)
        if drawEvery:
            animatePath(true)

    if 1:
        print 'Performed ',n,' predictions (every 20 frames) for the whole video: '
        print "Total average ERROR for predictions : "
        try:
            print sum(errorList) / len(errorList)
        except ZeroDivisionError:
            print 'Nothing in errorList'
        #print "Outlier list: "
        #print outlierPositionDic

if singleInvest:
    prediction, error, true = predict(singleInvest, 63, 0.6, 0, True)
    print 'error: ', error
    animatePath(true)
    animatePath(prediction)
if 1:
    prediction = predict(0, 63, 0.6, 1, True)
    print "#############The Predicted List#########"
    print prediction
    try:
        for i in range(len(prediction)):
            outputFile.write(str(prediction[i]))
            outputFile.write('\n')
        print "Prediction successfully saved in ./result folder!"
    except Exception as e:
        print 'Warning: can not save prediction list in file!',e
    print '#############Draw Prediction############'
    print "Now drawing the predicted path after video ends: "
    try:
        animatePath(prediction)
    except Exception as e:
        print "Warning:", e
        print 'Can not draw path on your computer, please check the "animatePath" function in util.py and make sure the required pacages are istalled;'
        print 'This warning does NOT affect the successful output of result!'
    outputFile.close()
######################################################################
'''
The underling code are used for outputting some of the detailed in formation when design the code

You CAN ignore them if you want

Otherwise, you can change each "if 0" line to "if 1" to see the output and check the functionality of the code

'''

###element for testing the hit finding condition, change if 0 to if 1 to exe:
if 0:
    j = 0
    k = 23.1  #1344/57
    hitOrder = []
    tmp = None
    mapping = {1000: '0', 1010: '02', 1001: '03', 100: '1', 110: '12', 101: '13', 1: '3', 10: '2'}
    for i in range(len(hitEdge)):
        if hitEdge[i] > 0:
            j += 1
            if hitEdge[i] != tmp:
                hitOrder.append(mapping[hitEdge[i]])
            if hitEdge[i] == 1000: print j, hitEdge[i], i, "X_low", '---', X_pos[i], "~", xmin, ',', Y_pos[
                i], '  ', i / k, 's'
            if hitEdge[i] == 1010: print j, hitEdge[i], i, "X_low/Y_low", '---', X_pos[i], "~", xmin, ',', Y_pos[
                i], '~', ymin, '  ', i / k, 's'
            if hitEdge[i] == 1001: print j, hitEdge[i], i, "X_low/Y_up", '---', X_pos[i], "~", xmin, ',', Y_pos[
                i], '~', ymax, '  ', i / k, 's'
            if hitEdge[i] == 100: print j, hitEdge[i], i, "X_up", '---', X_pos[i], "~", xmax, ',', Y_pos[
                i], '  ', i / k, 's'
            if hitEdge[i] == 110: print j, hitEdge[i], i, "X_up/Y_low", '---', X_pos[i], "~", xmin, ',', Y_pos[
                i], '~', ymin, '  ', i / k, 's'
            if hitEdge[i] == 101: print j, hitEdge[i], i, "X_up/Y_up", '---', X_pos[i], "~", xmin, ',', Y_pos[
                i], '~', ymax, '  ', i / k, 's'
            if hitEdge[i] == 1: print j, hitEdge[i], i, "Y_up", '---', X_pos[i], ',', Y_pos[
                i], '~', ymax, '  ', i / k, 's'
            if hitEdge[i] == 10: print j, hitEdge[i], i, "Y_low", '---', X_pos[i], ',', Y_pos[
                i], '~', ymin, '  ', i / k, 's'
            tmp = hitEdge[i]

    print hitOrder
    print 'hit time:', len(hitOrder)

    print X_pos
    print Y_pos
    print speed
    print speedChange
    print heading
    print turnAngle
    print hitEdge


###print error between KL-filter and actual data, execute if necessary
if 0:
    print '#############check x##########'
    measurements = X_pos[0:60]
    KalmanFilter(measurements)
    print '#############check y##########'
    measurements = Y_pos[0:60]
    KalmanFilter(measurements)

