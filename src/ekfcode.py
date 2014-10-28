from math import *
import random
import time
import turtle
import numpy as np

def angle_trunc(a):
    '''helper function to map all angles onto [-pi, pi]'''
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

###########################################################
### robot #################################################
###########################################################

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

    def __repr__(self):
        """This allows us to print a robot's position"""
        return '[%.5f, %.5f]'  % (self.x, self.y)

    def move_using(self,next_position_fn):
        next_pos = next_position_fn(self.x,self.y)
        next_heading = atan2(next_pos[1]-self.y,
                             next_pos[0]-self.x)
        self.turning = angle_trunc(next_heading - self.heading)
        self.distance = distance_between( (self.x,self.y) , next_pos )
        self.move(self.turning,self.distance)

#####################################################
### main ############################################
#####################################################

def parabolic_path(x0,y0):
    return (x0+1,
            y0+2*x0)

def parabolic_ekf(x,P,meas,var_meas=0,var_move=0):
    F = np.matrix([[1.,0.],
                   [2.,1.]]) # the matrix of partial derivatives
                             # of the motion model
                             # f(x,y) = (x+1,2x+y)
                             # dfx/dx = 1, dfx/dy = 0
                             # dfy/dx = 2, dfy/dy = 1
    H = np.matrix([[1.,0.],
                   [0.,1.]]) # the matrix of partial derivatives
                             # of the measurement model
                             # h(x,y) = (x,y)
                             # dhx/dx = 1, dhx/dy = 0
                             # dhy/dx = 0, dhy/dy = 1
    Q = np.matrix([[var_move,0.],
                   [0.,var_move]]) # movement covariance matrix
    R = np.matrix([[var_meas, 0.],
                   [ 0.,var_meas]]) # measurement covariance matrix
    I = np.matrix([[1.,0.],
                   [0.,1.]]) # identity matrix
    
    # measurement update
    z = np.matrix([[meas[0]],
                   [meas[1]]]) # the measurement we receive

    # if we had a different function to measure than just taking
    # h(x) = x, then this would be y = z - h(x)
    y = z - x

    # the rest of the equations are the same as in the regular KF!
    S = H * P * np.transpose(H) + R
    K = P * np.transpose(H) * np.linalg.inv(S)
    x = x + (K * y)
    P = (I - K * H) * P

    # the only real change is in the prediction update equations
    # instead of using a matrix to update the x vector, we use
    # the function f
    next_pt = parabolic_path(meas[0],meas[1])
    x = np.matrix([[next_pt[0]],
                   [next_pt[1]]])

    # this equation is the same as in the regular KF!
    P = F * P * np.transpose(F) + Q
        
    return x,P

def estimate_next_pos(target_meas,OTHER=None):
    if not OTHER:
        OTHER = {}
        #OTHER['meas'] = [target_meas]
        OTHER['x'] = np.matrix([[0.],
                                [0.]])
        OTHER['P'] = np.matrix([[1000.,    0.],
                                [   0., 1000.]])
    #else:
    #    OTHER['meas'].append(target_meas)
    
    OTHER['x'],OTHER['P'] = parabolic_ekf(OTHER['x'],OTHER['P'],target_meas,var_meas=0.2)
    x,y = np.transpose(OTHER['x']).tolist()[0]
    return (x,y), OTHER

def demo_grading(target_bot, estimate_fn, path_type, OTHER=None, visualize=False, max_steps=1000, tolerance_ratio = 0.01):
    caught = False
    steps = 0

    # begin visualization setup
    window = None
    broken_robot = None
    size_multiplier = 15.0
    measuredbroken_robot = None
    if visualize:
        window = turtle.Screen()
        window.bgcolor('white')
        broken_robot = turtle.Turtle()
        broken_robot.shape('turtle')
        broken_robot.color('green')
        broken_robot.resizemode('user')
        broken_robot.shapesize(0.3, 0.3, 0.3)
        broken_robot.hideturtle()
        broken_robot.penup()
        print target_bot.x
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        broken_robot.showturtle()
        measuredbroken_robot = turtle.Turtle()
        measuredbroken_robot.shape('circle')
        measuredbroken_robot.color('red')
        measuredbroken_robot.penup()
        measuredbroken_robot.resizemode('user')
        measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
        broken_robot.pendown()
    # end visualization setup

    while not caught:
        measurement = target_bot.sense()
        estimate,OTHER = estimate_fn(measurement,OTHER)
        target_bot.move_using(path_type)
        actual = (target_bot.x,target_bot.y)

        # begin visualization
        if visualize:
            #measuredbroken_robot.setheading(target_bot.heading*180/pi)
            measuredbroken_robot.goto(measurement[0]*size_multiplier, measurement[1]*size_multiplier-100)
            measuredbroken_robot.stamp()
            broken_robot.setheading(target_bot.heading*180/pi)
            broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        # end visualization

        # update steps taken and check for going over max
        steps += 1
        
        separation = distance_between(estimate, actual)
        if separation < tolerance_ratio * target_bot.distance:
            print "You got it right! It took you ", steps, " steps to locate the target."
            print 'estimated position =',estimate
            print 'actual position    =',actual
            print 'separation =',separation
            caught = True
            break

        if steps >= max_steps:
            print "It took too many steps to locate the target:", max_steps, "elapsed"
            print 'estimated position =',estimate
            print 'actual position    =',actual
            print 'separation =',separation
            break
            
        if steps % 100 == 0:
            print 'steps:',steps
            print 'estimated position =',estimate
            print 'actual position    =',actual
            print 'separation =',separation

    return caught

def run_test():
    ### feel free to play around with these parameters!
    params = { 'max_steps' : 1000,
               'tolerance_ratio': 0.001,
               'dist_noise': 0,
               'turn_noise': 0,
               'meas_noise': 2.0,
               'init_x': 0.,
               'init_y': 0. }

    target = robot(x=params['init_x'],y=params['init_y'])
    target.set_noise(params['turn_noise'],
                     params['dist_noise'],
                     params['meas_noise'])
    return demo_grading(target,estimate_next_pos,parabolic_path,
                 visualize=False,
                 max_steps=params['max_steps'],
                 tolerance_ratio=params['tolerance_ratio'])

run_test()
