#!/usr/bin/env python3

from time import sleep
import rospy

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class driveSquare(object):
    """ This node publishes ROS messages containing the 3D coordinates of a single point """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.sleep(1)
        for i in range(4):
            self.twist = Twist(linear=Vector3(0.5,0,0),angular=Vector3(0,0,0))
            self.robot_movement_pub.publish(self.twist)
            rospy.sleep(2)
            self.twist = Twist(linear=Vector3(0,0,0),angular=Vector3(0,0,1.7))
            self.robot_movement_pub.publish(self.twist)
            rospy.sleep(1)
            
        self.twist = Twist(linear=Vector3(0,0,0),angular=Vector3(0,0,0))
        self.robot_movement_pub.publish(self.twist)



if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = driveSquare()
    node.run()