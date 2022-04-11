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
min_dis = 0.5



class PersonFollower(object):
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

    def process_scan(self, data):
        #set a maximum distance that the robot will scan for and set a move variable that determines the degree distance of the closest object
        max_dis = 3.1
        move = -1
        
        
        for i in range (181):
            #does a full 360 degree scan and finds the closest object to the robot and determines its angle out of 180 degrees to the right or to the left
            r = - data.ranges[-i]
            
            l = data.ranges[i]

            if (abs(r)<abs(max_dis) and abs(r)>= min_dis):
                max_dis = r
                move = i
            if (l<abs(max_dis) and l>= min_dis):
                max_dis = l
                move = i
        #make sure the default speed if nothing is detected is 0
        self.twist.linear.x = 0
        
        #since max_dis is positive or negative depending on if the object is to the left or to the right, we isolate which direction we turn in
        if (max_dis < 0 and move >=0):
            self.twist.angular.z = -move/20
            print(max_dis)
        if (max_dis > 0 and move >=0):
            self.twist.angular.z = move/20
            print(max_dis)
        #only moves if something is detected within a ten degree radius in both directions 
        if (move < 10 and move >= 0):
            self.twist.linear.x = abs(max_dis)
        

            
        
        self.twist_pub.publish(self.twist)
        


    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = PersonFollower()
    node.run()
