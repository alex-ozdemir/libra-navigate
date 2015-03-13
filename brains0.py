
#
# brains0.py
#

import roslib; roslib.load_manifest('irobot_mudd')  # irobot_mudd
import rospy
import cv
import time
from std_msgs.msg import *
from sensor_msgs.msg import *
from irobot_mudd.srv import *
from irobot_mudd.msg import *
from math import *
from laser import HALLWAY_WIDTH_IN_MM
from nav_constants import *
import libra_map


# class for a generic data holder
class Data:  pass
D = Data()             # our global data object, D

BACK_UP_TIME = 4.2
MAX_ANGLE_DEV = 10
ROTATION_SCALE = 5
BASE_SPEED = 200
WALL_DIST_SCALE = BASE_SPEED / 5000.0 
WALL_ANGLE_SCALE = BASE_SPEED / 75.


# globals for incoming data streams...
D.list_laser_data = None
D.start_time = time.time() # start of program
D.last_time_printed = 0    # we'll print once in a while
D.last_time_clocked = D.start_time # the last time we hit the stopwatch
D.intersection_data = [False, True, True, False]



TURN_SPEED = BASE_SPEED / 2
TURN_TIME_360 = 8.30 * 100 / TURN_SPEED
STRAIGHT_TIME = 8
ONE_FOOT_TIME = 1.3 / (BASE_SPEED / 200.)
def ENTER_INTERSECTION_TIME():
    return int(ONE_FOOT_TIME * getInstructions()[2] / 2.)

D.STATE = "WAITING_TO_START"
# States!
"WAITING_TO_START"               # the very first state entered
"HALLWAY"
"ENTER_INTERSECTION"
"HANDLE_INTERSECTION"
"LEAVE_INTERSECTION"

D.NAV_STATE = 0

D.libra = libra_map.make_libra_small()

D.NAV_DATA = D.libra.get_directions('A', 'B', 'E')
print "Instructions:"
print D.NAV_DATA

def getInstructions():
    if D.NAV_STATE < len( D.NAV_DATA ):
        return D.NAV_DATA[D.NAV_STATE]
    else:
        return False


last_hall = None

def laser_data_callback(data):
    """ runs with each laser_data message from the laser-handling code """
    global D, last_hall
    message = data.data # that's the string
    D.list_laser_data = eval( message ) # Yay, Python!
    D.intersection_data = D.list_laser_data[8:12]
    
    if D.intersection_data != last_hall:
        print last_hall, '->', D.intersection_data
        last_hall = D.intersection_data

    if D.STATE == "HALLWAY":
        expected_intersection = getInstructions()[0]
        if expected_intersection:
            if D.intersection_data == expected_intersection:
                print "Expected Intersection detected!", D.intersection_data
                enter_state("ENTER_INTERSECTION")
        else:
            enter_state("WAITING_TO_START")

    elif D.STATE == "ENTER_INTERSECTION":
        front_distance = D.list_laser_data[5]
        #print 'dist to wall:', front_distance
        if front_distance < HALLWAY_WIDTH_IN_MM / 2:
            enter_state("HANDLE_INTERSECTION")

    # don't do lots of work here... instead, we'll do it all in main
    # we won't print it... if you want to see this stream use (in a new tab)
    # rostopic list
    # rostopic echo laser_data

def key_press_callback(data):
    """ runs with each message from the keyboard_data stream """
    global D
    message = data.data # that's the string
    D.last_keypress = message
    # we'll handle stuff here...
    k = D.last_keypress

    if k in ' ': 
        D.robot_publisher.publish( "toggle commands" ) # Wow!
    if k in 'W': # 'W' goes to the waiting state
        D.robot_publisher.publish( "D.tank(0,0)" ) # Yay, Python!
        D.STATE = "WAITING_TO_START"  # back to waiting to start


def robot_sensor_callback( data ):
    """ sensor_callback is called for each sensorPacket message
    """
    global D
    if data.wheeldropCaster == True:
        # enable this later!
        #D.robot_publisher.publish( "D.tank(0,0)" ) # Yay, Python!
        D.STATE = "WAITING_TO_START"  # back to waiting to start
        

    if data.advance == True:  
        #D.robot_publisher.publish( "D.tank(0,0)" ) # Yay, Python!
        D.STATE = "WAITING_TO_START"  # back to waiting to start
        
    if data.play == True:
        #D.robot_publisher.publish( "song()" ) # Yay, Python!
        # Start your state machine here, perhaps!
        D.STATE = "HALLWAY"

    if data.bumpRight or data.bumpLeft == True:
        #print "My Bumpers!"
        #D.last_time_clocked = time.time()
        #D.robot_publisher.publish( "D.tank(-100,100)" ) # Yay, Python!
        D.STATE = "WAITING_TO_START"
    # again, we won't print it, but you can use
    # rostopic list
    # rostopic echo sensor


def clock( current_time ):
    """ print once in a while """
    global D
    number_of_seconds_since_start = int(current_time - D.start_time)
    if D.last_time_printed < number_of_seconds_since_start:
        print "[Brains] [State:", D.STATE, "]  time is", \
              number_of_seconds_since_start, "seconds since starting..."
        D.last_time_printed = number_of_seconds_since_start


def follow_wall():
    # use D.list_laser_data to proportionally control our motors
    left_speed = BASE_SPEED
    right_speed = BASE_SPEED

    if D.list_laser_data[6]:
        left_speed += int(-WALL_ANGLE_SCALE * (D.list_laser_data[6]))
        right_speed += int(WALL_ANGLE_SCALE * (D.list_laser_data[6]))

    if D.list_laser_data[7]:
        left_speed += int(-WALL_DIST_SCALE * (D.list_laser_data[7]))
        right_speed += int(WALL_DIST_SCALE * (D.list_laser_data[7]))

    D.robot_publisher.publish( "D.tank(%d,%d)" %(left_speed, right_speed) ) 

def go_straight():
    left_speed = BASE_SPEED
    right_speed = BASE_SPEED
    D.robot_publisher.publish( "D.tank(%d,%d)" %(left_speed, right_speed) ) 

def do_turn(turn_type):
    if turn_type in [TURN_LEFT, TURN_RIGHT]:
        D.robot_publisher.publish( "D.tank(%d,%d)" %(-turn_type * TURN_SPEED, turn_type * TURN_SPEED) ) 
        return TURN_TIME_360 * 90 / 360
    elif turn_type == TURN_AROUND:
        D.robot_publisher.publish( "D.tank(%d,%d)" %(-TURN_SPEED, TURN_SPEED) ) 
        return TURN_TIME_360 * 180 / 360

def enter_state(state):
    D.last_time_clocked = time.time()
    D.STATE = state

def main():
    """ creates and displays a GUI for the range finder data
        Ranges window: shows range finder values as red lines
        coming from the center of the range finder
        HoughLines window: shows the result of using a Hough
        transformation on image containing the range values as points.
    """
    global D

    # initialize ROS subscription
    rospy.init_node("robot_brains", anonymous=True)

    # subscribers
    rospy.Subscriber( 'laser_data', String, laser_data_callback)
    rospy.Subscriber( 'sensorPacket', SensorPacket, robot_sensor_callback )
    rospy.Subscriber( 'keyboard_data', String, key_press_callback )

    # publishers
    D.robot_publisher = rospy.Publisher('robot_commands', String)
    D.state_publisher = rospy.Publisher('state_strings', String)

    # 
    # main loop - and state machine!
    #
    while rospy.is_shutdown() == False:
        time.sleep(0.04)  # a little pause for each loop...
        current_time = time.time() ; clock( current_time ) # print every second

        D.state_publisher.publish( D.STATE ) # in another tab: rostopic echo state_strings

        if D.STATE == "HALLWAY":
            follow_wall()

        elif D.STATE == "ENTER_INTERSECTION":
            go_straight()
            if current_time > ENTER_INTERSECTION_TIME() + D.last_time_clocked:
                enter_state("HANDLE_INTERSECTION")

        elif D.STATE == "HANDLE_INTERSECTION":
            if current_time >  do_turn(getInstructions()[1]) + D.last_time_clocked:
                enter_state("LEAVE_INTERSECTION")

        elif D.STATE == "LEAVE_INTERSECTION":
            #TODO: May be unstable
            follow_wall()
            if current_time > STRAIGHT_TIME + D.last_time_clocked:
                D.STATE = "HALLWAY"
                D.NAV_STATE += 1

        elif D.STATE == "WAITING_TO_START":
            D.robot_publisher.publish( "D.tank(0,0)" )

        else:
            print "I don't recognize the state", D.STATE


    print "Quitting the main loop..."
            
     
    
if __name__ == "__main__":
    main()
