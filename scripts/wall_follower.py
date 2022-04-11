#!/usr/bin/env python3


# TOPICS:
#   cmd_vel: publish to, used for setting robot velocity
#   scan   : subscribing, where the wall is

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# How close we will get to the person.
min_dis = 0.1



class WallFollower(object):
    """ This node walks the robot to wall and stops """

    def __init__(self):
        # Start rospy node.
        rospy.init_node("follow_person")
        
        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Get a publisher to the cmd_vel topic.
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)
        self.twist_pub.publish(self.twist)
        

    def process_scan(self, data):
        # Determine closeness to wall by looking at scan data from in front of
        #   the robot, set linear velocity based on that information, and
        #   publish to cmd_vel.

        # The ranges field is a list of 360 number where each number
        #   corresponds to the distance to the closest obstacle from the
        #   LiDAR at various angles. Each measurement is 1 degree apart.

        # The first entry in the ranges list corresponds with what's directly
        #   in front of the robot.
        
        left = 0
        turn = 0.4
        iturn = -50
        safety = 1
        isafety = -50
        

       
        
        for i in range (181):
            left = data.ranges[i]
            if (left < turn and left >= min_dis):
                turn = left 
                iturn = i
                
            if (left < safety and left >= min_dis):
                safety = left 
                isafety = i

        print(iturn)

        self.twist.angular.z = (iturn - 90) / 100 + ((turn - 0.25)*2)

        if (iturn >= 100):
            self.twist.angular.z = (iturn - 90) / 10 + ((turn - 0.25)*20)
        
        self.twist.linear.x = 0.05

        
        if (iturn == -50):
            self.twist.angular.z = 0
            self.twist.linear.x = 0.1
            if (isafety >= 30):
                self.twist.angular.z = safety * 10
                self.twist.linear.x = 0.001/ (1+ abs(90-isafety))
            
        
        print(self.twist.linear.x)
        # Publish msg to cmd_vel.
        ##rospy.sleep(100)
        self.twist_pub.publish(self.twist)
        


    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = WallFollower()
    node.run()