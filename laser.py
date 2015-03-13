#!/usr/bin/python
#############################################
##    Original authors: Cyrus Huang, Zakkai Davidson
##    Freed from the Neato and rewritten by Adam Dunlap!
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
from Vector import Vector
from WallBuffer import WallBuffer

# class for a generic data holder
class Data:  pass
D = Data()         # our global data object, D

# window variables
WIN_WIDTH  = 600                # keeps square windows
WIN_HEIGHT = WIN_WIDTH
SIZE       = (WIN_WIDTH,WIN_HEIGHT)
CENTER     = WIN_WIDTH/2

HALLWAY_WIDTH_IN_MM = 2134
ALLOWED_DEVIATION = 45

NO_WALL_DIST = 1.25*(HALLWAY_WIDTH_IN_MM)
SEARCH_WIDTH = 15

# line drawing variables
MAX_MAG      = 10000             # unreliable data above 10m
MAG_SCALE    = 100              # 100 pixels per meter
ANGLE_OFFSET = -90              # front of robot faces up on screen
REV          = 360              # 360 degrees per rev (range data is stored in degrees)

# options
SHOW_HOUGH = False              # set to true to show Hough transformation image

BUFFER_SIZE = 10



def init_GUI():
    """ initializes open cv windows and creates images to display """
    global D

    print
    print "Press 'q' in Ranges window to quit"
    
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

    # initialize ROS subscription
    rospy.init_node("range_listener", anonymous=True)

    # subscribe to laser data
    rospy.Subscriber('lidar', neato_lidar.msg.LidarRanges, range_callback)

    # storage needed for Hough processing
    D.storage = cv.CreateMemStorage(0)
    
    # give initial values to range data before first callback
    D.ranges =[0]*REV

    # storage for publishing
    D.to_publish = [0]*12

    D.wall_buffers = [WallBuffer(BUFFER_SIZE, False), WallBuffer(BUFFER_SIZE, True), WallBuffer(BUFFER_SIZE, True), WallBuffer(BUFFER_SIZE, False)]



def findWalls():
    """returns a four-element boolean list with the wall-values in NEWS order"""
    global D

    wall_data = [isWall( (ang - SEARCH_WIDTH)%360, (ang + SEARCH_WIDTH)%360 ) \
                                 for ang in [0, 270, 90, 180]]

    D.to_publish[8:12] = [D.wall_buffers[i].submit(wall_data[i]) for i in xrange(len(wall_data))]


def isWall(angle_1, angle_2):
    """returns true if the angle range given represents a wall"""
    global D

    #print "before: ", D.ranges
    all_ranges = D.ranges
    #print "after: ", ranges

    if angle_1 > angle_2:
        ranges = all_ranges[angle_1 - 1 : ] + all_ranges[ : angle_2]
    else:
        ranges = all_ranges[angle_1 - 1 : angle_2]

    ranges.sort()

    median = ranges[len(ranges)/2]

    #print "For angle ", angle_1 + SEARCH_WIDTH, " the median is ", median
    #print "For angle ", angle_1 + SEARCH_WIDTH, " the ranges are ", ranges

    return median < NO_WALL_DIST


def range_callback(data):
    """ callback to handle ranges """
    global D
    D.ranges = list(data.ranges)
    for i in xrange(len(D.ranges)):
        if D.ranges[i] == 0:
            D.ranges[i] = MAX_MAG + 1


def handle_key_presses():
    """ does just that! """
    global D

    # wait and check for quit request
    key_code = cv.WaitKey(1) & 255    # numeric value is key_code
    if key_code == 255:
        return
    key_press = chr(key_code)         # string vaue is key_press

    if ' ' <= key_press <= 'z': # if it's in our valid range
        print "Publishing ", str(key_press)
        D.kbd_pub.publish(String(str(key_press))) # publish!

    # check for ESC or 'q'
    if key_code == 27 or key_press == 'q' : # if ESC or 'q' was pressed
        time.sleep(0.1)  # pause for a moment...    # then, shutdown:
        rospy.signal_shutdown( "Quitting..." )

    if 32 <= key_code < 128:  
        print "the key_press was", key_press, key_code


    # add more key-press behaviors here...

    return


def polarToCartesian(angle, distance):

    if distance > MAX_MAG:
        return (CENTER, CENTER)

    adjustedAngle = radians(angle + ANGLE_OFFSET)
    distInMeters = distance/1000.0
    pixelDistance = distInMeters*MAG_SCALE

    xCoordinate = pixelDistance*sin(adjustedAngle)
    yCoordinate = pixelDistance*cos(adjustedAngle)

    col = int(CENTER + xCoordinate)
    row = int(CENTER - yCoordinate)

    return (row, col)




def draw_laser_ranges():
    """ this function should
          (1) draw to the D.image window the ranges as rays
              the walls (once computed), and
              their midpoints
          (2) it should also draw to the D.hough image the dots
              needed as input to the Hough transformation
    """

    global D

    if True:  # for easy commenting out...
        NUM_RANGES = len(D.ranges) # should be 360
        for angle in range(NUM_RANGES):
            #print angle, ":", D.ranges[angle] 

            # add line corresponding to this angle reading
            endPoint = polarToCartesian(angle, D.ranges[angle])
            cv.Line(D.image, (CENTER, CENTER), endPoint, cv.RGB(255, 0, 0), 1)

            # add point corresponding to this angle reading
            cv.Line(D.hough, endPoint, endPoint, 255, 2)
         
    # helpful starting points, perhaps:
    # add line to the ranges image, "D.image"
    #cv.Line(D.image, (42,100), (100,42), cv.RGB(255, 0, 0), 1) # 1 == thickness
    # add dots to image being used to compute the Hough tr. "D.hough"
    #cv.Line(D.hough, (42,42), (42,42), 255, 2) # 1 == thickness



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
    distance_res = 20 # old value 3  # distance resolution of Hough accumulator, in pixels
    angle_res    = radians(10)  # angular resolution of Hough acc., in radians
    min_votes    = 60#20 # how many votes needed to count a line?
    min_line_len = 0.05 * MAG_SCALE # shortest allowable line, in pixels
    max_gap_len  = 50 # pixels
    
    # The return value is a list of line segments
    Lines = cv.HoughLines2(D.hough,
                           D.storage,
                           cv.CV_HOUGH_PROBABILISTIC,
                           distance_res, 
                           angle_res,
                           min_votes,
                           min_line_len,
                           max_gap_len)

    Lines = filter(lambda l: sqrt((l[0][0] - l[1][0]) ** 2 + (l[0][1] - l[1][1]) ** 2) \
        > min_line_len, Lines)

    if len(Lines) > 0:
        best_front = best_front_line(Lines)
        best_left = best_left_line(Lines)
        best_right = best_right_line(Lines)

        (right_angle, right_distance) = angle_and_x_distance(best_right)
        (left_angle, left_distance) = angle_and_x_distance(best_left)
        (front_angle, front_distance) = angle_and_y_distance(best_front)
        
        D.to_publish[0:8] =  (right_angle, left_angle, front_angle, right_distance, left_distance, front_distance, 
                          safe_avg(right_angle, left_angle), safe_center_dist(right_distance, left_distance))

        for line in [best_front, best_left, best_right]:
            if line:
                start_pt = line[0]
                end_pt = line[1]
                midpoint = ((start_pt[0] + end_pt[0]) / 2,(start_pt[1] + end_pt[1]) / 2)
                #print angle_0_to_90(line)

                cv.Line(D.image, start_pt, end_pt, cv.RGB(0, 0, 255), 5) # 1 == thickness

    N = len(Lines)
    for i in range(N):
        line = Lines[i]
        
        start_pt = line[0]
        end_pt = line[1]
        midpoint = ((start_pt[0] + end_pt[0]) / 2,(start_pt[1] + end_pt[1]) / 2)

        cv.Line(D.image, start_pt, end_pt, cv.RGB(0, 255, 0), 1) # 1 == thickness
        cv.Line(D.image, midpoint, midpoint, cv.RGB(255, 255, 0), 4)

def safe_avg(right_angle, left_angle):
    if right_angle:
        if left_angle:
            return (right_angle + left_angle)/2
        else: 
            return right_angle
    return left_angle

def safe_center_dist(right_distance, left_distance):
    right_center_dist = None
    left_center_dist = None
    if right_distance:
        right_center_dist = (HALLWAY_WIDTH_IN_MM)/2 - right_distance
    if left_distance:
        left_center_dist = left_distance - (HALLWAY_WIDTH_IN_MM)/2
    return safe_avg(right_center_dist, left_center_dist)


def angle_and_x_distance(line):
    if line:
        vector = (Vector(line[1]) - Vector(line[0]))
        if vector.angle() < 0:
            vector = - vector
        angle = 90 - degrees(vector.angle())
        distance = int(1000. / MAG_SCALE * abs(CENTER - (line[0][0] + line[1][0])/2))
    else:
        angle = None
        distance = None
    return (angle, distance)

def angle_and_y_distance(line):
    if line:
        vector = (Vector(line[1]) - Vector(line[0]))
        if vector.angle() < 0:
            vector = - vector
        angle = 90 - degrees(vector.angle())
        distance = int(1000. / MAG_SCALE * abs(CENTER - (line[0][1] + line[1][1])/2))
    else:
        angle = None
        distance = None
    return (angle, distance)

def best_right_line(lines):

    lines_ = filter(lambda l: (l[0][0] + l[1][0]) / 2 > CENTER, lines)
    lines_ = filter(lambda l: (90 - angle_0_to_90(l)) < ALLOWED_DEVIATION, lines_)
    lines_.sort(lambda l1, l2: angle_0_to_90(l2) - angle_0_to_90(l1))
    return lines_[0] if len(lines_) > 0  else None


def best_left_line(lines):

    lines_ = filter(lambda l: (l[0][0] + l[1][0]) / 2 < CENTER, lines)
    lines_ = filter(lambda l: (90 - angle_0_to_90(l)) < ALLOWED_DEVIATION, lines_)
    lines_.sort(lambda l1, l2: angle_0_to_90(l2) - angle_0_to_90(l1))
    return lines_[0] if len(lines_) > 0  else None

def best_front_line(lines):

    lines_ = filter(lambda l: (l[0][1] + l[1][1]) / 2 < CENTER, lines)
    lines_ = filter(lambda l: angle_0_to_90(l) < ALLOWED_DEVIATION, lines_)
    lines_.sort(lambda l1, l2: angle_0_to_90(l1) - angle_0_to_90(l2))
    return lines_[0] if len(lines_) > 0  else None

def angle_0_to_90(line):
    return int(degrees(pi/2 - abs(pi/2 - slope_angle(line) % pi)))

def slope_angle(l):
    return atan2(l[0][1] - l[1][1], l[0][0] - l[1][0])

def init_publishers():

    # create a String publisher (pub)
    # you will be able to subscribe to it via the name 'wall_data'
    D.PUB = rospy.Publisher('laser_data',String)

    # create a String publisher (pub), for KBD data
    D.kbd_pub = rospy.Publisher('keyboard_data',String)

def main():
    """ creates and displays a GUI for the range finder data
        Ranges window: shows range finder values as red lines
        coming from the center of the range finder
        HoughLines window: shows the result of using a Hough
        transformation on image containing the range values as points.
    """
    global D
    
    init_GUI() # initialize images and windows

    init_publishers()
    
    # main loop
    while rospy.is_shutdown() == False:

        # handle any pending keypresses... - need to keep this
        handle_key_presses()
        
        # draw the ranges in D.ranges to the screen
        draw_laser_ranges()

        # find walls and add to image using Hough transformation
        findHoughLines()

        # detect boolean walls in NEWS directions
        findWalls()

        D.PUB.publish(str(D.to_publish))


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
