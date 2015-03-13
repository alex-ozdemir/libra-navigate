#
# robot.py
#

#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import irobot_mudd
#from ControllerParser import update_D # don't worry about this until part 2
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import time
import math


"""

to start the robot
WITH bluetooth:

    hcitool scan
    sudo rfcomm connect 0 00:0A:3A:2E:CB:3E 1
    rosparam set /irobot_mudd/port /dev/rfcomm0; sudo chmod 777 /dev/rfcomm0; rosrun irobot_mudd driver.py

WITH serial cable:
    plug in the laser first!! (it will be /dev/ttyUSB0) THEN run this line
    rosparam set /irobot_mudd/port /dev/ttyUSB1; sudo chmod 777 /dev/ttyUSB1; rosrun irobot_mudd driver.py

"""


####
# D is our global system state
####

class Data: pass    # empty class for a generic data holder
D = Data()  # an object to hold our system's services and state
D.start_time = time.time()
D.ignore_commands = False   # do we ignore incoming commands?



# some global lists for joystick control...
D.BTNS = [0]*52
D.PADS = [0]*52


    
def song():
    # sing!
    D.song([76,76,30,76,30,72,76,30,79], # notes
           [15,15,15,15,15,15,15,15,15]) # durations
  
####
# handle_robot_commands ~ called with each message from the 'robot_commands' stream...
####
def handle_robot_commands(data):
    """ This function is called for each published message
    """
    global D
    message = data.data
    if message == "toggle commands":   # special messages...
        D.ignore_commands = not D.ignore_commands
        if D.ignore_commands == True:
            D.tank(0,0) # stop if you'd like to ignore...
        return 

    print "I received the string", message,
    if D.ignore_commands == False:
        eval( message ) # Yay, Python!
    else:
        print "[ignoring...]",
    print




def sensor_callback( data ):
    """ sensor_callback is called for each sensorPacket message
    """
    global D

    leftfront = data.cliffFrontLeftSignal
    rightfront = data.cliffFrontRightSignal
    left = data.cliffLeftSignal
    right = data.cliffRightSignal

    # not doing much with this... we'll let brains.py do that



def main():
    """ returns an object (tank) that allows you
       to set the velocities of the robot's wheels
    """
    global D # to hold system state

    # we need to give our program a ROS node name
    # the name is not important, so we use "lab1_node"
    rospy.init_node('robot_node', anonymous=True)
    
    # 
    # services!
    #
    # we obtain the tank, song, and LEDs services...
    rospy.wait_for_service('tank') # wait until the motors are available
    D.tank = rospy.ServiceProxy('tank', Tank) # D.tank is our "driver"
    rospy.wait_for_service('song') # wait until our voice is available
    D.song = rospy.ServiceProxy('song', Song) # D.song is our "speaker" 
    rospy.wait_for_service('leds')
    D.leds = rospy.ServiceProxy('leds', Leds)

    # subscribe to the robot_commands data!
    rospy.Subscriber( 'robot_commands', String, handle_robot_commands )

    # set up a callback for the sensorPacket stream, i.e., "topic"
    # Note that this should really be used only for very low-level things...
    rospy.Subscriber( 'sensorPacket', SensorPacket, sensor_callback )

    # 
    # main loop - and state machine!
    #
    while rospy.is_shutdown() == False:

        time.sleep(1)  # a little pause for each loop...
        current_time = time.time()
        time_diff = int( current_time - D.start_time)
        print "[Robot] awake for", time_diff, "seconds"




    print "Quitting the robot's main loop..."




####
# It all starts here...
#
# This is the "main" trick: it tells Python what code to run
# when you execute this file as a stand-alone script:
#### 

if __name__ == "__main__":
   main()