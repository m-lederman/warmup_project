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

##max_dis = 3.1

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
        # Determine closeness to wall by looking at scan data from in front of
        #   the robot, set linear velocity based on that information, and
        #   publish to cmd_vel.

        # The ranges field is a list of 360 number where each number
        #   corresponds to the distance to the closest obstacle from the
        #   LiDAR at various angles. Each measurement is 1 degree apart.

        # The first entry in the ranges list corresponds with what's directly
        #   in front of the robot.
        #lvel = 0
        #rvel = 0
        max_dis = 3.1
        move = 0
        
        
        for i in range (180):
            r = - data.ranges[-i]
            
            l = data.ranges[i]

            if (abs(r)<abs(max_dis) and abs(r)>= min_dis):
                max_dis = r
                move = i
            if (l<abs(max_dis) and l>= min_dis):
                max_dis = l
                move = i
        self.twist.linear.x = 0
        if (max_dis < 0):
            self.twist.angular.z = -move/20
            print(max_dis)
        if (max_dis > 0):
            self.twist.angular.z = move/20
            print(max_dis)
        if (move < 10 and move > 0):
            self.twist.linear.x = abs(max_dis)
        

            
          ##  if (right == 0 or left == 0 or right > max_dis or left > max_dis):
          ##      self.twist.angular.z = 0
                
       ##     if (right > 0 and right <= max_dis and (left == 0 or left > max_dis)):
       ##         self.twist.angular.z = -i/5
       ##         print("r")
       ##         print(i)
                
        ##        break
            
        ##    if (left > 0 and left <= max_dis and (right == 0 or right > max_dis)):
        ##        self.twist.angular.z = i/5
         ##       print("l")
        ##        print(i)
              ##  print(2)
         ##       break
         ##   if (data.ranges[180] > 0 and data.ranges[180] <= max_dis):
         ##       self.twist.angular.z = 30 
            ##print(self.twist.angular.z)
            
       ## for i in range(11):
            
       ##     lvel = data.ranges[i]
            
       ##     rvel = data.ranges[-i]
           
         #   if ((lvel >= min_dis and lvel < max_dis) or (rvel >= min_dis and rvel < max_dis)):
                # Go forward if not close enough to person.
      #          self.twist.linear.x = lvel / 2 + rvel /2
       #         break
       #     if ((lvel == 0.0 or lvel >= max_dis or lvel <= min_dis) and (rvel == 0.0 or rvel >= 3.1 or rvel <= min_dis)) :
                ##self.twist.angular.z = (right + left) * 0.5
        #        self.twist.linear.x = 0
        
        ##print(self.twist.linear.x)
        # Publish msg to cmd_vel.
        ##rospy.sleep(100)
        self.twist_pub.publish(self.twist)
        


    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = PersonFollower()
    node.run()