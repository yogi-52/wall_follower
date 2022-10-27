#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#Defining variables 

max_range = 3.5  #Max range of the laser.
min_range = 0.12  #Min range of the laser.

max_angle = 3.142  #Max angle of the laser scanner
min_angle = -3.142  #Min angle of the laser scanner

angle_increment = 0.008  #Angle increment

speed = [0.08, 0.12]  #Speeds


def posCallback(msg):

    global f, b, l, r, ray #, f_r, b_r, f_l, b_l

    ray = msg.ranges

    f = msg.ranges[len(msg.ranges)//2]  #Front readings
    r = msg.ranges[len(msg.ranges)//4]  #Right readings
    l = msg.ranges[len(msg.ranges)*3//4]  #Left readings
    b = msg.ranges[len(msg.ranges) - 1]  #Back readings

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
    #print("Rotating left")
    vel.linear.x = 0
    vel.angular.z = 0.4

def rotateRight():
    vel.linear.x = 0
    vel.angular.z = -0.4

def stopRobot():
    vel.linear.x = 0
    vel.angular.z = 0


def wallFollow():

    if max_range > f > 0.8:
        print("front clear, move fast")
        moveForward(speed[1])
        if 0.3 > r > 0.2:
            print("Follow wall, move straight")
            moveForward(speed[1])
        elif max_range > r > 0.3:  #If obstacle at right is farther than 0.3, the robot must move closer towards it.
            print("Move closer to wall")
            turnCW()
        elif 0.2 < r < max_range: #If obstacle at right is closer than 0.2, the robot must move away.
            print("Move farther from the wall")
            turnCCW()      
        
    elif f < 0.8:
        print("wall close, move slow")
        moveForward(speed[0])  #Move slow.
        turnCCW()  #Turn CCW to avoid collision with front wall

        if f <= 0.3:  #If obstacle at front is less thn 0.5, the robot must turn left till obstacle at right is between 0.2 and 0.3.
            print("Too close! move reverse")  
            moveBackwards(speed[0])  #Go reverse
    pub.publish(vel)
    rate.sleep()


if __name__ == '__main__':
    try:
        vel = Twist()  #Making vel object of Twist msg type
        rospy.init_node('wall_follow')
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size= 1)
        sub = rospy.Subscriber('/scan', LaserScan, posCallback)
        rate = rospy.Rate(20)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node Terminated")