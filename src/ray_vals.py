#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan


# This function will print the current ray values(ie distances) in 8 directions.
def callback(msg):

        
    print(f"f= {msg.ranges[len(msg.ranges)//2]}")  # Front ray: 360th
    print(f"r= {msg.ranges[len(msg.ranges)//4]}")  # Right ray: 180th
    print(f"l= {msg.ranges[len(msg.ranges)*3//4]}")  # Left ray: 540th
    print(f"b= {msg.ranges[len(msg.ranges) - 1]}")  # Back ray: 719th ray
    print("\n")
    print(f"f_r= {msg.ranges[len(msg.ranges)*3//8]}")  # Front_Right ray: 270th
    print(f"b_r {msg.ranges[len(msg.ranges)//8]}")  # Back_Right ray: 90th
    print(f"f_l= {msg.ranges[len(msg.ranges)*5//8]}")  # Front-Left ray: 450th
    print(f"b_l= {msg.ranges[len(msg.ranges)*7//8]}")  # Back_left ray: 630th
    print("\n\n")


if __name__ == '__main__':
    try:
        rospy.init_node('scan_values')
        sub = rospy.Subscriber('/scan', LaserScan, callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node Terminated")