#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_follower.srv import FindWall, FindWallRequest

# Defining variables 

max_range = 3.5  # Max range of the laser.
min_range = 0.12  # Min range of the laser.

max_angle = 3.142  # Max angle of the laser scanner
min_angle = -3.142  # Min angle of the laser scanner

angle_increment = 0.008  # Angle increment

speed = [0.08, 0.12]  # Speeds


def servClient():
    rospy.wait_for_service('/find_wall')
    find_wall_client = rospy.ServiceProxy('/find_wall', FindWall)
    find_wall_request_obj = FindWallRequest()
    result = find_wall_client(find_wall_request_obj)

    print(result)

def posCallback(msg):
    global f, b, l, r, ray # , f_r, b_r, f_l, b_l

    ray = msg.ranges

    f = msg.ranges[len(msg.ranges)//2]  # Front readings
    r = msg.ranges[len(msg.ranges)//4]  # Right readings
    l = msg.ranges[len(msg.ranges)*3//4]  # Left readings
    b = msg.ranges[len(msg.ranges) - 1]  # Back readings

    
    wallFollow()


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


# This function makes the robot follow the wall, keeping it to the right side.
def wallFollow():
    if max_range > f > 0.8:  # Follow the wall fast, if it's farther than 0.8m
        moveForward(speed[1])
        if 0.3 > r > 0.2:  # Keep following fast if the right is correctly aligned.
            moveForward(speed[1])
        elif max_range > r > 0.3:  # If wall at right is farther than 0.3,
            turnCW()               # the robot must move closer towards it.            
        elif 0.2 < r < max_range: # If wall at right is closer than 0.2,
            turnCCW()             # the robot must move away.
     
    elif f < 0.8:  # If the wall is closer than 0.8 meters, move slow!
        moveForward(speed[0])  
        turnCCW()  #  Turn CCW to avoid collision with front wall

        if f <= 0.3:  # If wall at front is less than 0.3,
                      # the robot must back up!
            moveBackwards(speed[0])  # Go reverse
    pub.publish(vel)
    rate.sleep()


if __name__ == '__main__':
    try:
        vel = Twist()
        rospy.init_node('wall_follow_client')
        servClient()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size= 1)
        sub = rospy.Subscriber('/scan', LaserScan, posCallback)
        rate = rospy.Rate(20)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node Terminated")