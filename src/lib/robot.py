from math import *
import random,sys
from util import *

# helper function to map all angles onto [-pi, pi]
def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

class robot:

    def __init__(self, x = 0.0, y = 0.0, heading = 0.0, turning = 2*pi/10, distance = 1.0):
        """This function is called when you create a new robot. It sets some of
        the attributes of the robot, either to their default values or to the values
        specified when it is created."""
        self.x = x
        self.y = y
        self.heading = heading
        self.turning = turning # only applies to target robots who constantly move in a circle
        self.distance = distance # only applies to target bot, who always moves at same speed.
        self.turning_noise    = 0.0
        self.distance_noise    = 0.0
        self.measurement_noise = 0.0


    def set_noise(self, new_t_noise, new_d_noise, new_m_noise):
        """This lets us change the noise parameters, which can be very
        helpful when using particle filters."""
        self.turning_noise    = float(new_t_noise)
        self.distance_noise    = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)

    def set_heading(self, new_heading):
        self.heading = new_heading

    def set_turning(self, new_turning):
        self.turning = new_turning

    def set_distance(self, new_distance):
        self.distance = new_distance

    def move(self, turning, distance, tolerance = 0.001, max_turning_angle = pi):
        """This function turns the robot and then moves it forward."""
        # apply noise, this doesn't change anything if turning_noise
        # and distance_noise are zero.
        turning = random.gauss(turning, self.turning_noise)
        distance = random.gauss(distance, self.distance_noise)

        # truncate to fit physical limitations
        turning = max(-max_turning_angle, turning)
        turning = min( max_turning_angle, turning)
        distance = max(0.0, distance)

        # Execute motion
        self.heading += turning
        self.heading = angle_trunc(self.heading)
        self.x += distance * cos(self.heading)
        self.y += distance * sin(self.heading)

    def move_in_circle(self):
        """This function is used to advance the runaway target bot."""
        self.move(self.turning, self.distance)

    def sense(self):
        """This function represents the robot sensing its location. When
        measurements are noisy, this will return a value that is close to,
        but not necessarily equal to, the robot's (x, y) position."""
        return (random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise))

    def sense_all(self):
        """This function represents the robot sensing its location. When
        measurements are noisy, this will return a value that is close to,
        but not necessarily equal to, the robot's (x, y) position."""
        return [random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise),self.distance,self.heading,self.turning,]

    def __repr__(self):
        """This allows us to print a robot's position"""
        return '[%.5f, %.5f]'  % (self.x, self.y)


def getDirec(id, X_pos,Y_pos,quiet=True):
    if id > 0 and id <len(X_pos):
        if X_pos[id] == X_pos[id-1] and Y_pos[id]==Y_pos[id-1]:
            if not quiet:
                print 'warning: same data point:'
                print id,'point1: ' ,X_pos[id],Y_pos[id],' point2:  ',X_pos[id-1],Y_pos[id-1]
            return False
        initSpeed = distance_between((X_pos[id],Y_pos[id]),(X_pos[id-1],Y_pos[id-1]))
        direc = None
        try:
            direc = acos((X_pos[id]-X_pos[id-1])/initSpeed)
        except ZeroDivisionError:
            if not quiet:
                print 'warning: zero division at getDirec: '
                print id,'point1: ' ,X_pos[id],Y_pos[id],' point2:  ',X_pos[id-1],Y_pos[id-1]
        if (Y_pos[id]-Y_pos[id-1])<0:
            direc = -1*direc
        return direc
    else:
        return 0

def getTurn(id, X_pos,Y_pos):
    if len(X_pos)==len(Y_pos):
        if (id+1) < len(X_pos) and id>=0:
            dir1 = getDirec(id,X_pos,Y_pos)
            if dir1 is False:
                dir1 = getDirec(id-1,X_pos,Y_pos)
            dir2 = getDirec(id+1,X_pos,Y_pos)
            if dir2 is False:
                return 0
            return dir2-dir1
        else:
            return 0
    else:
        print 'getTurn error: list lengths differ'
        return False

def getSpeed(id, X_pos, Y_pos,quiet =True):
    if id > 0 and id <len(X_pos):
        if X_pos[id] == X_pos[id-1] and Y_pos[id]==Y_pos[id-1]:
            if not quiet:
                print 'warning: same data point:'
                print id,'point1: ' ,X_pos[id],Y_pos[id],' point2:  ',X_pos[id-1],Y_pos[id-1]
            return False
        initSpeed = distance_between((X_pos[id],Y_pos[id]),(X_pos[id-1],Y_pos[id-1]))
        return initSpeed
    else:
        return 0

def getSpeedChange(id, X_pos,Y_pos):
    if len(X_pos)==len(Y_pos):
        if (id+1) < len(X_pos) and id>=0:
            dir1 = getSpeed(id,X_pos,Y_pos)
            if dir1 is False:
                dir1 = getSpeed(id-1,X_pos,Y_pos)
            dir2 = getSpeed(id+1,X_pos,Y_pos)
            if dir2 is False:
                return 0
            return dir2-dir1
        else:
            return 0
    else:
        print 'getTurn error: list lengths differ'
        return False