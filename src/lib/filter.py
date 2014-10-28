__author__ = 'mahandong'
from matrix import *
from util import *
from robot import *

u = matrix([[0.], [0.]]) # external motion
F = matrix([[1., 1.], [0, 1.]]) # next state function
H = matrix([[1., 0.]]) # measurement function
R = matrix([[1.]]) # measurement uncertainty
I = matrix([[1., 0.], [0., 1.]]) # identity matrix


# Implement the Kalman filter function below, from assignment
def filter(x, P, measurements, quiet=True):
    predict_pos = []
    predict_vol = []
    for n in range(len(measurements)):
        # measurement update
        j = H * x
        y = matrix([[measurements[n]]])- j #y: 1*1
        a = H*P*H.transpose() #a: 1*1
        S = a + R #S: 1*1  ##a.inverse()
        z = P*H.transpose() #z: 2*1
        k = z*S.inverse() #k: 2*1
        b = k*y #b: 2*1
        x = x + b
        P = (I - k*H)*P
        # prediction
        x = F*x + u
        P = F*P*F.transpose()
        if not quiet:
            print 'x= '
            x.show()
            print 'P= '
            P.show()
        predict_pos.append(x.value[0])
        predict_vol.append(x.value[1])
    return predict_pos, predict_vol


def KalmanFilter(measurements):
    x = matrix([[measurements[0]], [0]]) # initial state (location and velocity)
    P = matrix([[10., 0.], [0., 1000.]]) # initial uncertainty

    predict_pos_x, predict_vol_x = filter(x, P, measurements)
    predict_pos_x = [item for sublist in predict_pos_x for item in sublist]
    predict_pos_x_int = [int(x) for x in predict_pos_x]
    print 'predict: '
    print predict_pos_x_int
    print 'reality: '
    print measurements
    print 'error:'
    print calculateError1D(predict_pos_x_int,measurements)


def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the bot
    based on (x, y) measurements."""
    if OTHER == None:
        OTHER = {'xy':measurement} #for trackin the previous (x,y)
        xy_estimate = (0.0,0.0)
    else:
        #calculate the heading of current bot
        distance = distance_between(OTHER['xy'],measurement) #distance between the 2 points
        heading = acos((measurement[0]-OTHER['xy'][0])/distance)
        if (measurement[1]-OTHER['xy'][1])<0:
            heading = -1*heading

        #print measurement, OTHER['xy'], heading*180/pi
        # Finding the turning direction
        if 'heading' not in OTHER.keys():
            turning = None
            xy_estimate = (0., 0.)
        else:
            turning = heading - OTHER['heading']

            dummy = robot(measurement[0], measurement[1], heading, turning, distance)
            dummy.set_noise(0., 0., 0.)
            dummy.move_in_circle()
            xy_estimate = (dummy.x, dummy.y)
        OTHER['heading'] = heading
        OTHER['xy'] = measurement
    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    return xy_estimate, OTHER
