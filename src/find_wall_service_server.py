#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_follower.srv import FindWall, FindWallResponse


# Defining variables

max_range = 3.5  # Max range of the laser.
min_range = 0.12  # Min range of the laser.

max_angle = 3.142  # Max angle of the laser scanner
min_angle = -3.142  # Min angle of the laser scanner

angle_increment = 0.008  # Angle increment

speed = [0.08, 0.12]  # Speed options: slow and fast.
#ray = []


def servCallback(req):
    #global ray

    fw_service_response = FindWallResponse()  # Defining response object.
    rospy.loginfo("/find_wall service called")
    min1 = ray[0]
    val = findWall(min1)
    result = mainBehaviour(val)
    fw_service_response.wallfound = result
    
    if fw_service_response.wallfound:
        rospy.loginfo("Behaviour achieved")
    else:
        rospy.loginfo("Behaviour failed!")

    return fw_service_response


def posCallback(msg):

    global f, b, l, r, ray, f_r, min1

    ray = msg.ranges
    
    f = msg.ranges[len(msg.ranges)//2]  # Front readings
    r = msg.ranges[len(msg.ranges)//4]  # Right readings
    l = msg.ranges[len(msg.ranges)*3//4]  # Left readings
    b = msg.ranges[len(msg.ranges) - 1]  # Back readings

    f_r = ray[len(ray)*3//8]  #Front-Right readings (ray number 270)


def moveForward(spd):
    vel.linear.x = spd
    vel.angular.z = 0


def moveBackwards(spd):
    vel.linear.x = -spd
    vel.angular.z = 0


def turnCW():
    vel.linear.x = 0.1
    vel.angular.z = -0.4


def turnCCW():
    vel.linear.x = 0.1
    vel.angular.z = 0.4


def rotateLeft():
    vel.linear.x = 0
    vel.angular.z = 0.4


def rotateRight():
    vel.linear.x = 0
    vel.angular.z = -0.4


def stopRobot():
    vel.linear.x = 0
    vel.angular.z = 0


# This function will find the wall, by finding the lowest ray value.
def findWall(min1):
    # find the minimum ray dist index
    for i in range(len(ray)-1):
        if ray[i] < min1:
            min1 = ray[i]
            # time.sleep(0.5)

    return min1

# This function will move the robot's front closer to the wall.
def alignFront(val):
    aligned = False  # Is the robot aligned? -> NO
    while not (val+0.05) >= f >= (val-0.05):  # while the front is not almost equal to the
                                              # minimum ray distance, rotate the robot to right.
        rotateRight()
        pub.publish(vel)
        rate.sleep()

        if (val+0.05) >= f >= (val-0.05):
            aligned = True  # Is the robot aligned? -> YES
            break

    stopRobot()
    pub.publish(vel)
    rate.sleep()
    
    return aligned

# This function willl align the robot's front right to face the wall.   
def alignFrontRight(val):
    aligned = False
    while not (val+0.05) >= f_r >= (val-0.05):
        rotateLeft()
        pub.publish(vel)
        rate.sleep()
        if (val+0.05) >= f_r >= (val-0.05):
            aligned = True
            break
    stopRobot() 
    pub.publish(vel)
    rate.sleep()

    return aligned


# This function executes the main expected behavior for the robot
# ie, once the wall is found, it will align its front move closer,
# then align its front-right(ray 270) to the wall, and return True if aligned correctly.
def mainBehaviour(val):

    align_obj = alignFront(val)
    while align_obj:
        if f > 0.3:
            moveForward(speed[0])
        elif f <= 0.3:
            stopRobot()
            break
        pub.publish(vel)
        rate.sleep()
    align_f_r_obj = alignFrontRight(val)

    return align_f_r_obj


if __name__ == '__main__':
    try:
        vel = Twist()
        rospy.init_node('wall_follow')
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        sub = rospy.Subscriber('/scan', LaserScan, posCallback)
        fin_wall_service = rospy.Service('/find_wall', FindWall, servCallback)
        rospy.loginfo("Service online.")
        rate = rospy.Rate(20)
        rospy.spin()

    except rospy.ROSInterruptException():
        rospy.loginfo("Node Terminated")
