#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge
import json
import rospy
import actionlib
import threading
from std_msgs.msg import String
from pkg_task5.msg import myMessage
#from rospy_message_converter import message_converter
from pkg_ros_iot_bridge.msg import msgRosIotAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult      # Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback    # Message Class that is used for Feedback Messages    

from pkg_ros_iot_bridge.msg import msgMqttSub           # Message Class for MQTT Subscription Messages

from pyiot import iot                                   # Custom Python Module to perfrom MQTT Tasks

goal_handle={'protocol':'mqtt','mode':'pub','topic':'eyrc/RpAaSgSa/ros_to_iot'}
goal_handle_1={'protocol':'mqtt','mode':'pub','topic':'eyrc/RpAaSgSa/ros_to_iot'}

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
        
         
    
    
    def on_goal(self,gol_handle):
        """
        This function will be called when Action Server receives a Goal.
        Parameters : gol_handle is a dictionary containing all data recieved from obj_msg
        """
        # Validate incoming goal parameters
        if(gol_handle['protocol'] == "mqtt"):
            
            if((gol_handle['mode'] == "pub") or (gol_handle['mode'] == "sub")):
                #goal_handle.set_accepted()
                
                # Start a new thread to process new goal from the client (For Asynchronous Processing of Goals)
                # 'self.process_goal' - is the function pointer which points to a function that will process incoming Goals
                thread = threading.Thread(  name="worker",
                                            target=self.process_goal,
                                            args=(gol_handle,) )
                thread.start()

            else:
                gol_handle.set_rejected()
                return
        
        else:
            gol_handle.set_rejected()
            return 

    def process_goal(self,gol_handle):
        """
        It is used for further processing of gol_handle.
        Parameters : gol_handle is a dictionary passed by on_goal function.
        """
        # Goal Processing
        result = msgRosIotResult()
        if(gol_handle['protocol'] == "mqtt"):
            rospy.logwarn("MQTT")
            print('inside dict....')
            #print(goal_handle['x'])
            if(gol_handle['mode'] == "pub"):
                
                ret = iot.mqtt_publish( self._config_mqtt_server_url, 
                                        self._config_mqtt_server_port,
                                        self._config_mqtt_pub_topic, 
                                        gol_handle['pkg_dit'] ,
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
                                                        gol_handle.topic, 
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
            
            print("gol_handle = "+str(gol_handle))
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            
    def on_cancel(self, gol_handle):
        """
        This function will be called when Goal Cancel request is send to the Action Server.
        """
        rospy.loginfo("Received cancel request.")
        goal_id = gol_handle.get_goal_id()


def func_callback_topic_my_topic(obj_msg):
    """
    Callback function for recieving online orders and shipping.
    Parameters : obj_msg is in string form.
    """
    print("callback of online order and shipped ")

    # rospy.loginfo("Data Received: (%d, %d, %d )", obj_msg.final_x,
    #               obj_msg.final_y, obj_msg.final_theta)
    # goal_handle['x']=obj_msg.final_x
    # goal_handle['y']=obj_msg.final_y
    # goal_handle['theta']=obj_msg.final_theta
    # action_server.on_goal(goal_handle)
    print(type(obj_msg.pkg_dit))

    print(obj_msg.pkg_dit)
    data = eval(obj_msg.pkg_dit)
    goal_handle['pkg_dit']= data
    # #data = message_converter.convert_ros_message_to_dictionary(obj_msg)
    # msg = obj_msg.read()
    # data = json.loads(msg)
    # print("Type",type(data))
    
    # print("00000")
    #print(data)
    print(goal_handle)
    action_server.on_goal(goal_handle)
    ###### print("11111")


def func_callback_topic_dispatch_topic(obj_msg):
    """
    Callback function for orders dispatched.
    Parameters : obj_msg is in string form.
    """
    print("callback for dispatch ")
    print(type(obj_msg.pkg_dit))

    print(obj_msg.pkg_dit)
    data = eval(obj_msg.pkg_dit)
    goal_handle_1['pkg_dit']= data
    # #data = message_converter.convert_ros_message_to_dictionary(obj_msg)
    # msg = obj_msg.read()
    # data = json.loads(msg)
    # print("Type",type(data))
    
    # print("00000")
    #print(data)
    print(goal_handle_1)
    action_server.on_goal(goal_handle_1)

def my_listener_iot():
    """
    This function is used to subscribe to the desired topic and attach a Callback Funtion to it.
    """

    # 2. Subscribe to the desired topic and attach a Callback Funtion to it.
    #rospy.Subscriber("my_topic",msgTurtleResult , func_callback_topic_my_topic)
    rospy.Subscriber("my_topic", myMessage, func_callback_topic_my_topic)
    rospy.Subscriber("dispatch_topic", myMessage, func_callback_topic_dispatch_topic)
    # 3. spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()





# Main
def main():
    rospy.init_node('node_iot_ros_bridge_action_server',anonymous=True)
    global action_server
    action_server = IotRosBridgeActionServer()
    
    my_listener_iot()
    
    rospy.spin()



if __name__ == '__main__':
    rospy.sleep(25)
    main()
