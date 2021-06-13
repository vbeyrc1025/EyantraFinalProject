#!/usr/bin/env python
import rospy
import actionlib
import time

from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_task1.msg import msgTurtleResult
from pkg_task1.msg import msgTurtleAction

from pkg_task1.msg import msgTurtleGoal         # Message Class that is used for Goal messages
     
  
global flg_first



class SimpleActionClientTurtle:

    # Constructor
    def __init__(self):
        self._ac = actionlib.SimpleActionClient('/action_turtle',
                                               msgTurtleAction )
        self._ac.wait_for_server()
        rospy.loginfo("Action server is up, we can send new goals!")
        global flg_no
        flg_no=1
        
        param_config_pyiot = rospy.get_param('config_pyiot')
        self._config_mqtt_sub_cb_ros_topic = param_config_pyiot['mqtt']['sub_cb_ros_topic']

        rospy.Subscriber(self._config_mqtt_sub_cb_ros_topic, msgMqttSub, func_callback_topic_my_topic)

        #self._var_handle_pub = rospy.Publisher('eyrc/RpAaSgSa/ros_to_iot',msgTurtleResult, queue_size=10)

     # Function to send Goals to Action Servers
    def send_goal(self, arg_dis, arg_angle):
        
        # Create Goal message for Simple Action Server
        goal = msgTurtleGoal(distance=arg_dis, angle=arg_angle)
        
        '''
            * done_cb is set to the function pointer of the function which should be called once 
                the Goal is processed by the Simple Action Server.

            * feedback_cb is set to the function pointer of the function which should be called while
                the goal is being processed by the Simple Action Server.
        ''' 
        self._ac.send_goal(goal, done_cb=self.done_callback,
                           feedback_cb=self.feedback_callback)
        
        rospy.loginfo("Goal has been sent.")
        
    # Function print result on Goal completion
    def done_callback(self, status, result):
        rospy.loginfo("Status is : " + str(status))
        rospy.loginfo("Result is : " + str(result))
       
        global flag_publish
        flag_publish = True 
         # 1. Create a handle to publish messages to a topic.
        var_handle_pub = rospy.Publisher('my_topic',msgTurtleResult, queue_size=10)
    
        
    # 3. Set the Loop Rate 
        var_loop_rate = rospy.Rate(0.5) # 1 Hz : Loop will its best to run 1 time in 1 second
        
        if not rospy.is_shutdown():
            obj_msg = msgTurtleResult()

            obj_msg.final_x= result.final_x  
            obj_msg.final_y = result.final_y
            obj_msg.final_theta = result.final_theta
            if flag_publish == True:
                rospy.loginfo("Publishing: ")
                rospy.loginfo(obj_msg)
                flag_publish = False

            
            var_handle_pub.publish(result)

            var_loop_rate.sleep()
            #break
   
                
       
    # Function to print feedback while Goal is being processed
    def feedback_callback(self, feedback):
        rospy.loginfo(feedback)

def func_callback_topic_my_topic(myMsg):

    rospy.loginfo("Data Received: (%s)", myMsg.message)
    if myMsg.message=="start":
        
        obj_client.send_goal(2, 0)
        rospy.sleep(5)
        obj_client.send_goal(2, 60)
        rospy.sleep(15)
        obj_client.send_goal(2, 60)
        rospy.sleep(15)
        obj_client.send_goal(2, 60)
        rospy.sleep(15)
        obj_client.send_goal(2, 60)
        rospy.sleep(15)
        obj_client.send_goal(2, 60)
        rospy.sleep(15)


# Main Function
def main():
    # 1. Initialize ROS Node
    rospy.init_node('node_simple_action_client_turtle',anonymous=True)
    
    # 2. Create a object for Simple Action Client.
    global obj_client
    obj_client = SimpleActionClientTurtle()
    #my_listener_client()
    
    rospy.spin()
    

if __name__ == '__main__':
    main()

