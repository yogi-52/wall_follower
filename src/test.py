#! /usr/bin/env python

#import time
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# Defining variables

max_range = 3.5  # Max range of the laser.
min_range = 0.12  # Min range of the laser.

max_angle = 3.142  # Max angle of the laser scanner
min_angle = -3.142  # Min angle of the laser scanner

angle_increment = 0.008  # Angle increment

speed = [0.08, 0.12]  # Speeds


def posCallback(msg):

    global f, b, l, r, ray  # , f_r, b_r, f_l, b_l

    ray = msg.ranges

    f = msg.ranges[len(msg.ranges)//2]  # Front readings
    r = msg.ranges[len(msg.ranges)//4]  # Right readings
    l = msg.ranges[len(msg.ranges)*3//4]  # Left readings
    b = msg.ranges[len(msg.ranges) - 1]  # Back readings

    val = findwall()
    mainBehaviour(val)
    # wallFollowReady()
    # rightAlignToWall(val)


# Defining movement functions
def moveForward(spd):
    vel.linear.x = spd
    vel.angular.z = 0


def moveBackwards(spd):
    vel.linear.x = -spd
    vel.angular.z = 0


def turnCW():
    vel.linear.x = 0.1
    vel.angular.z = -0.4


def rotateLeft():
    print("Rotating left")
    vel.linear.x = 0
    vel.angular.z = 0.4


def rotateRight():
    vel.linear.x = 0
    vel.angular.z = -0.4


def turnCCW():
    vel.linear.x = 0.1
    vel.angular.z = 0.4


def stopRobot():
    print("Stop Robot")
    vel.linear.x = 0
    vel.angular.z = 0


def wallFollow():

    if max_range > f > 0.8:
        print("front clear, move fast")
        moveForward(speed[1])
        if 0.3 > r > 0.2:
            print("Follow wall, move straight")
            moveForward(speed[1])
        # If obstacle at right is farther than 0.3, the robot must move closer towards it.
        elif max_range > r > 0.3:
            print("Move closer to wall")
            turnCW()
        # If obstacle at right is closer than 0.2, the robot must move away.
        elif 0.2 < r < max_range:
            print("Move farther from the wall")
            turnCCW()

    elif f < 0.8:
        print("wall close, move slow")
        moveForward(speed[0])  # Move slow.
        turnCCW()  # Turn CCW to avoid collision with front wall

        if f <= 0.3:  # If obstacle at front is less thn 0.5, the robot must turn left till obstacle at right is between 0.2 and 0.3.
            print("Too close! move reverse")
            moveBackwards(speed[0])  # Go reverse

    pub.publish(vel)
    rate.sleep()


def findwall():
    min1 = ray[0]
    # find the minimum ray dist index
    for i in range(len(ray)-1):
        if ray[i] < min1:
            min1 = ray[i]
            # time.sleep(0.5)

    return min1


def mainBehaviour(val):

    # rotate left
    rotateLeft()
    # once f == lowest, move forward.
    print(f, val, val + 0.05, val - 0.05, "CCCCCCCCCC")
    if (val+0.05) >= f >= (val-0.05):
        moveForward(speed[0])
        # once f < 0.3, STOP
        if f <= 0.3:
            stopRobot()
        else:
            pass

    pub.publish(vel)
    rate.sleep()


'''
def rightAlignToWall(val):

    rotateRight()
    if (val-0.05) < r < (val+0.05):
        print("right aligned to wall")
        stopRobot()

    if 0.3 >= r >= 0.2:
        print("right aligned")
    else:
        print("failed") 

    pub.publish(vel)
    rate.sleep()
'''


def rightReady():

    while r > 0.3:
        rotateRight()
    print("aligned")
    stopRobot()


def wallFollowReady():
    val = findwall()
    goToWall2(val)


def goToWall2(val):
    print(f"val recieved is {val}")
    #print("I should roatate left now")
    rotateLeft()
    if (val+0.05) > f > (val-0.05):
        print("front lowest found")
        if f > 0.3:
            moveForward(speed[0])
            print("f > 0.3")
        else:
            print("f<3")
    else:
        print("Did not execute forward motion")
    # stopRobot()
    print("Stopped the robot")

    pub.publish(vel)
    rate.sleep()


if __name__ == '__main__':
    try:
        vel = Twist()
        rospy.init_node('wall_follow')
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        sub = rospy.Subscriber('/scan', LaserScan, posCallback)
        rate = rospy.Rate(20)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node Terminated")
