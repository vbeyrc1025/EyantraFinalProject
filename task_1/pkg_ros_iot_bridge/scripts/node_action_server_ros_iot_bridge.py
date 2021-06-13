#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge

import rospy
import actionlib
import threading
from pkg_task1.msg import msgTurtleResult 
from pkg_ros_iot_bridge.msg import msgRosIotAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult      # Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback    # Message Class that is used for Feedback Messages    

from pkg_ros_iot_bridge.msg import msgMqttSub           # Message Class for MQTT Subscription Messages

from pyiot import iot                                   # Custom Python Module to perfrom MQTT Tasks

goal_handle={'protocol':'mqtt','mode':'pub','topic':'eyrc/RpAaSgSa/ros_to_iot','x':0,'y':0,'theta':0}

class IotRosBridgeActionServer:

    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_iot_ros',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        '''
            * self.on_goal - It is the fuction pointer which points to a function which will be called
                             when the Action Server receives a Goal.

            * self.on_cancel - It is the fuction pointer which points to a function which will be called
                             when the Action Server receives a Cancel Request.
        '''

        # Read and Store IoT Configuration data from Parameter Server
        param_config_pyiot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_pyiot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_pyiot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_pyiot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_pyiot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_pyiot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_pyiot['mqtt']['sub_cb_ros_topic']
        print(param_config_pyiot)


        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages from MQTT Subscription.
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10)


        # Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is defined in 'config_iot_ros.yaml'.
        # self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(  self.mqtt_sub_callback, 
                                                        self._config_mqtt_server_url, 
                                                        self._config_mqtt_server_port, 
                                                        self._config_mqtt_sub_topic, 
                                                        self._config_mqtt_qos   )
        if(ret == 0):
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")


        # Start the Action Server
        self._as.start()
        
        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    
    # This is a callback function for MQTT Subscriptions
   #   *argv
    def mqtt_sub_callback(self, client, userdata, message):
        payload = str(message.payload.decode("utf-8"))
    
        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)

        
        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload
        
        self._handle_ros_pub.publish(msg_mqtt_sub)
        
         
    
    # This function will be called when Action Server receives a Goal
    def on_goal(self,goal_handle):
        # Validate incoming goal parameters
        if(goal_handle['protocol'] == "mqtt"):
            
            if((goal_handle['mode'] == "pub") or (goal_handle['mode'] == "sub")):
                #goal_handle.set_accepted()
                
                # Start a new thread to process new goal from the client (For Asynchronous Processing of Goals)
                # 'self.process_goal' - is the function pointer which points to a function that will process incoming Goals
                thread = threading.Thread(  name="worker",
                                            target=self.process_goal,
                                            args=(goal_handle,) )
                thread.start()

            else:
                goal_handle.set_rejected()
                return
        
        else:
            goal_handle.set_rejected()
            return 

    def process_goal(self,goal_handle):
        # Goal Processing
        result = msgRosIotResult()
        if(goal_handle['protocol'] == "mqtt"):
            rospy.logwarn("MQTT")
            
            if(goal_handle['mode'] == "pub"):
                
                ret = iot.mqtt_publish( self._config_mqtt_server_url, 
                                        self._config_mqtt_server_port,
                                        self._config_mqtt_pub_topic, 
                                        goal_handle['x'],goal_handle['y'],goal_handle['theta'] ,
                                        self._config_mqtt_qos   )

                if(ret == 0):
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif(goal.mode == "sub"):
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(  self.mqtt_sub_callback, 
                                                        self._config_mqtt_server_url, 
                                                        self._config_mqtt_server_port, 
                                                        goal_handle.topic, 
                                                        self._config_mqtt_qos   )
                if(ret == 0):
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        rospy.loginfo("Send goal result to client")
        if (result.flag_success == True):
            rospy.loginfo("Succeeded")
            
            print("goal_handle = "+str(goal_handle))
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            
       
    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()


def func_callback_topic_my_topic(obj_msg):

    rospy.loginfo("Data Received: (%d, %d, %d )", obj_msg.final_x,
                  obj_msg.final_y, obj_msg.final_theta)
    goal_handle['x']=obj_msg.final_x
    goal_handle['y']=obj_msg.final_y
    goal_handle['theta']=obj_msg.final_theta
    action_server.on_goal(goal_handle)

def my_listener_iot():


    # 2. Subscribe to the desired topic and attach a Callback Funtion to it.
    rospy.Subscriber("my_topic",msgTurtleResult , func_callback_topic_my_topic)

    # 3. spin() simply keeps python from exiting until this node is stopped
    rospy.spin()





# Main
def main():
    rospy.init_node('node_iot_ros_bridge_action_server',anonymous=True)
    global action_server
    action_server = IotRosBridgeActionServer()
    
    my_listener_iot()
    
    rospy.spin()



if __name__ == '__main__':
    main()
