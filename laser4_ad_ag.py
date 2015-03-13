#!/usr/bin/python
#############################################
##    Original authors: Cyrus Huang, Zakkai Davidson
##    Freed from the Neato and rewritten by Adam Dunlap!
##    This one is by Adam and Alec Griffith :-)
##
##    NOTE:
##    Run laser driver first with
##
##    sudo chmod 777 /dev/ttyUSB* ; rosrun neato_lidar driver.py
#############################################
import roslib; roslib.load_manifest('neato_lidar')
import rospy
import cv
import time
import neato_lidar
import neato_lidar.msg

from std_msgs.msg import *
from sensor_msgs.msg import *
from math import *

# class for a generic data holder
class Data:  pass
D = Data()         # our global data object, D

# window variables
WIN_WIDTH  = 1000                # keeps square windows
WIN_HEIGHT = WIN_WIDTH
SIZE       = (WIN_WIDTH,WIN_HEIGHT)
CENTER     = WIN_WIDTH/2


# line drawing variables
MAX_MAG      = 1000             # unreliable data above 10m
MAG_SCALE    = 100              # 100 pixels per meter
ANGLE_OFFSET = -90              # front of robot faces up on screen
REV          = 360              # 360 degrees per rev (range data is stored in degrees)

# options
SHOW_HOUGH = False              # set to true to show Hough transformation image



def init_GUI():
    """ initializes open cv windows and creates images to display """
    global D

    print
    print "Press 'q' in Ranges window to quit"

    # initialize ROS subscription
    rospy.init_node("range_listener", anonymous=True)
    
    # create window and image to show range values
    cv.NamedWindow("Ranges")
    cv.MoveWindow("Ranges", WIN_WIDTH/2, WIN_HEIGHT/2)
    D.image = cv.CreateImage(SIZE, 8, 3) # 8 is pixel depth and 3 is # of channels

    # window for Hough transformation
    if SHOW_HOUGH:
        cv.NamedWindow("HoughInput")
        cv.MoveWindow("HoughInput", WIN_WIDTH/2 + WIN_WIDTH, WIN_HEIGHT/2)
        
    # image for Hough transformation
    D.hough = cv.CreateImage((WIN_WIDTH,WIN_HEIGHT), 8, 1) # image used for Hough transformation

    # subscribe to laser data
    rospy.Subscriber('lidar', neato_lidar.msg.LidarRanges, range_callback)
    D.wallPub = rospy.Publisher('laser_data', String)

    # storage needed for Hough processing
    D.storage = cv.CreateMemStorage(0)
    
    # give initial values to range data before first callback
    D.ranges =[0]*REV




def range_callback(data):
    """ callback to handle ranges """
    global D
    D.ranges = data.ranges



def handle_key_presses():
    """ does just that! """
    global D

    # wait and check for quit request
    key_code = cv.WaitKey(1) & 255    # numeric value is key_code
    key_press = chr(key_code)         # string vaue is key_press

    # check for ESC or 'q'
    if key_code == 27 or key_press == 'q' : # if ESC or 'q' was pressed
        time.sleep(0.1)  # pause for a moment...    # then, shutdown:
        rospy.signal_shutdown( "Quitting..." )

    if 32 <= key_code < 128:  
        print "the key_press was", key_press, key_code
        D.keyboard_pub.publish( key_press )



    # add more key-press behaviors here...

    return


def draw_laser_ranges():
    """ this function should
          (1) draw to the D.image window the ranges as rays
              the walls (once computed), and
              their midpoints
          (2) it should also draw to the D.hough image the dots
              needed as input to the Hough transformation
    """
    NUM_RANGES = len(D.ranges) # should be 360
    if False: #for easy commenting out...
        for angle in range(NUM_RANGES):
            print angle, ":", D.ranges[angle] 
         
    # helpful starting points, perhaps:
    # add line to the ranges image, "D.image"
    #cv.Line(D.image, (42,100), (100,42), cv.RGB(255, 0, 0), 1) # 1 == thickness
    # add dots to image being used to compute the Hough tr. "D.hough"
    # cv.Line(D.hough, (42,42), (42,42), 255, 2) # 1 == thickness
    for angle in range(NUM_RANGES):
        point = (CENTER + int(0.2*D.ranges[angle]*sin(radians(angle))), CENTER + int(0.2*D.ranges[angle]*cos(radians(angle))))
        cv.Line(D.image, (CENTER,CENTER), point, cv.RGB(255, 0 , 0), 1)
        cv.Line(D.hough, point, point, 255, 2) 

    return



def findHoughLines():
    """ Uses the Hough transformation to find lines from the sensor
        readings and displays them
    """
    global D

    # apply Hough transformation to find straight line segments
    # For more information, see:
    # http://docs.opencv.org/doc/tutorials/imgproc/imgtrans/hough_lines/hough_lines.html
    #
    # Here are the  options for the Hough transformation
    distance_res = 3  # distance resolution of Hough accumulator, in pixels, was 3
    angle_res    = radians(10)  # angular resolution of Hough acc., in radians, was 10
    min_votes    = 20 # how many votes needed to count a line?, was 20
    min_line_len = 25 # shortest allowable line, in pixels, was 5
    max_gap_len  = 40 # pixels, was 30
    
    # The return value is a list of line segments
    Lines = cv.HoughLines2(D.hough,
                           D.storage,
                           cv.CV_HOUGH_PROBABILISTIC,
                           distance_res, 
                           angle_res,
                           min_votes,
                           min_line_len,
                           max_gap_len)

    N = len(Lines)
    #print "\n\n"
    most_vertr_slope = 0
    most_vertr_line = ()
    most_vertl_slope = 0
    most_vertl_line = ()

    most_horiz_slope = float('inf')
    most_horiz_line = ()

    for i in range(N):
        line = Lines[i]
        #print "line[",i,"] is", line

        start_pt = line[0]
        end_pt = line[1]

        midpoint = ((start_pt[0] + end_pt[0])/2, (start_pt[1] + end_pt[1])/2)

        # helpful calls, perhaps:
        cv.Line(D.image, start_pt, end_pt, cv.RGB(0, 255, 0), 1) # 1 == thickness
        cv.Line(D.image, midpoint, midpoint, cv.RGB(0, 0, 255), 4)

        run = start_pt[0] - end_pt[0]
        if (run != 0):
            slope = (start_pt[1] - end_pt[1])/float(run)
        else:
            slope = float('inf')

        if abs(slope) > abs(most_vertl_slope) and start_pt[0] > CENTER and end_pt[0] > CENTER \
            and ((start_pt[1] < CENTER and end_pt[1]) > CENTER or (start_pt[1] > CENTER and end_pt[1] < CENTER)):
            most_vertl_slope = slope
            most_vertl_line = line
        if abs(slope) > abs(most_vertr_slope) and start_pt[0] < CENTER and end_pt[0] < CENTER \
            and ((start_pt[1] < CENTER and end_pt[1]) > CENTER or (start_pt[1] > CENTER and end_pt[1] < CENTER)):
            most_vertr_slope = slope
            most_vertr_line = line
        if abs(slope) < abs(most_horiz_slope) and start_pt[1] > CENTER and end_pt[1] > CENTER \
            and ((start_pt[0] < CENTER and end_pt[0]) > CENTER or (start_pt[0] > CENTER and end_pt[0] < CENTER)):

            most_horiz_slope = slope
            most_horiz_line = line
        #print line, slope

    if most_vertr_line:
        cv.Line(D.image, most_vertr_line[0], most_vertr_line[1], cv.RGB(255, 255, 255), 3) # 1 == thickness
    if most_vertl_line:
        cv.Line(D.image, most_vertl_line[0], most_vertl_line[1], cv.RGB(255, 255, 255), 3) # 1 == thickness
    if most_horiz_line:
        cv.Line(D.image, most_horiz_line[0], most_horiz_line[1], cv.RGB(255, 0, 100), 3) # 1 == thickness


    mpwrang = -1 if not most_vertr_line else -atan2(most_vertr_line[0][0]-most_vertr_line[1][0], most_vertr_line[0][1] - most_vertr_line[1][1])
    mpwlang = -1 if not most_vertl_line else -atan2(most_vertl_line[0][0]-most_vertl_line[1][0], most_vertl_line[0][1] - most_vertl_line[1][1])
    mpwfang = -1 if not most_horiz_line else -atan2(most_horiz_line[0][0]-most_horiz_line[1][0], most_horiz_line[0][1] - most_horiz_line[1][1])

    mpwrdist = -1 if not most_vertr_line else -most_vertr_line[0][0] + CENTER
    mpwldist = -1 if not most_vertl_line else +most_vertl_line[0][0] - CENTER
    mpwfdist = -1 if not most_horiz_line else most_horiz_line[0][1] - CENTER

    data_to_publish = (mpwrang, mpwlang, mpwfang, mpwrdist, mpwldist, mpwfdist)
    D.wallPub.publish(String(str( data_to_publish )))



def main():
    """ creates and displays a GUI for the range finder data
        Ranges window: shows range finder values as red lines
        coming from the center of the range finder
        HoughLines window: shows the result of using a Hough
        transformation on image containing the range values as points.
    """
    global D
    
    init_GUI() # initialize images and windows
    D.keyboard_pub = rospy.Publisher('keyboard_data',String)
    
    # main loop
    while rospy.is_shutdown() == False:

        # handle any pending keypresses... - need to keep this
        handle_key_presses()
        
        # draw the ranges in D.ranges to the screen
        draw_laser_ranges()

        # find walls and add to image using Hough transformation
        findHoughLines()

        # show color image with range finder data and calculated walls
        cv.ShowImage("Ranges",  D.image)
        
        # show b/w image used as the input to the Hough transformation, if desired
        if SHOW_HOUGH:  cv.ShowImage("HoughInput", D.hough) 

        # clear the images for next loop
        cv.Set(D.image, cv.RGB(0, 0, 0))
        cv.Set(D.hough, cv.RGB(0, 0, 0))

    print "Quitting..."
            
     

    
if __name__ == "__main__":
    main()
