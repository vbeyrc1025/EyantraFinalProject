#!/usr/bin/env python
#Script to move Turtle in a circle

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

    
def move_in_circle():

    #Publisher is created which is talker and can talk to turtle in turtlesim to move
    rospy.init_node('node_turtle_revolve', anonymous=True)
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
    pub.publish(Twist)
    
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
   
    # Create a Twist message and give suitable linear x and angular z velocity values
    move_cmd = Twist()
    move_cmd.linear.x = 0.5	#linear x velocity value
    move_cmd.angular.z = 1	#angular z velocity value

   
    

    # Save current time and set publish rate at 10 Hz
    rate = rospy.Rate(10)
    now = rospy.Time.now()

    # For the duration 6.1 seconds publish cmd_vel move commands to Turtlesim node
    while rospy.Time.now() < now + rospy.Duration.from_sec(6.1):
        pub.publish(move_cmd)
        rate.sleep()
   

def pose_callback(msg):
    rospy.loginfo("Turtle is moving in circle")	#prints the string when turtle is moving 
    print("Theta :"+str(msg.theta))		#prints the value of theta
    print(msg.x,msg.y)				#prints current position of turtle

if __name__ == '__main__':
    try:
        move_in_circle()
	rospy.loginfo("Goal is achieved")
    except rospy.ROSInterruptException:
        pass
