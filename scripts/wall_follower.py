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

# the closest we can get to the wall.
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
        
        #we set the variable left that takes in data points
        #we set turn to make sure our robot operates properly so long as it stays with 0.4 meters of the wall, which safety operates as a fail safe if the robot strays too far
        #iturn and isafety are default set to negative numbers so that we know if the robot does not detect anything at all
        left = 0
        turn = 0.4
        iturn = -50
        safety = 1
        isafety = -50
        

       
        #we only consider objects to the left of the robot, and we try to find the closest point detected
        for i in range (181):
            left = data.ranges[i]
            if (left < turn and left >= min_dis):
                turn = left 
                iturn = i
                
            if (left < safety and left >= min_dis):
                safety = left 
                isafety = i

       
        #we want our robot to turn so that it approaches a point where it is completely parallel to the wall and 0.25 meters away from it
        self.twist.angular.z = (iturn - 90) / 100 + ((turn - 0.25)*2)

        #if the closest wall detected to the robot is behind it, turn at a much quicker pace, since the most frequent case where the wall is closest behind is when there is nothin in front and so it needs to do a 90 degree or more turn in the open
        if (iturn >= 100):
            self.twist.angular.z = (iturn - 90) / 10 + ((turn - 0.25)*20)
        
        #the robot goes forward at a constant speed
        self.twist.linear.x = 0.05

        #if nothing is detected between 0.1 and 0.4 meters, then either the robot moves forward until it finds something or if it detects something within a meter that isn't head on, it tries to move towards that object as a correction mechanism
        if (iturn == -50):
            self.twist.angular.z = 0
            self.twist.linear.x = 0.1
            if (isafety >= 30):
                self.twist.angular.z = safety * 10
                self.twist.linear.x = 0.001/ (1+ abs(90-isafety))
            
        
       
        # Publish msg to cmd_vel.
        
        self.twist_pub.publish(self.twist)
        


    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = WallFollower()
    node.run()
