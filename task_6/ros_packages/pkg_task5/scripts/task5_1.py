#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import operator

import threading
import yaml
import os
import math
import time
import sys
import copy
import datetime

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from pkg_task5.msg import myMessage
import paho.mqtt.client as mqtt
#from rospy_message_converter import message_converter

import cv2
import json
#from pyiot import iot
from pkg_ros_iot_bridge.msg import msgMqttSub
import numpy as np
from pkg_vb_sim.srv import conveyorBeltPowerMsg 
from std_srvs.srv import Empty
from pkg_vb_sim.srv import vacuumGripper

def empty(a):
  """
  This is just a empty function and it is called when we are trying to detect the color of packages.
  """
  pass


class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):

        rospy.init_node('node_moveit_eg7', anonymous=True)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._group.set_planning_time(60)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''


        # Attribute to store computed trajectory by the planner 
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories_shelf/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)

        self.flagg = "True"
        print(self.flagg)

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    

    def set_joint_angles(self, arg_list_joint_angles):

        """ 
        This function is used for ur5 arm to reach the specific joint angles.
        Parameters : A list of joint angles to be set for the arm.
        Returning type : Returns flag_plan as True on successfully setting joint angles. 
        """

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        """ 
        The hard_set_joint_angles functions performs the certain attempts to make set_joint_angles run successfully. 
        Parameters : A list of joint angles to be set for the arm
                     Maximum number of attempts to be execute set_joint_angles function.  
        Returning type : None
        """

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()

    

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):

        """ 
        Function moveit_play_planned_path_from_file is used to play the trajectories saved in config folder. 
        Parameters : Path of the .yaml file to be played
                     Name of the .yaml file to be executed
        """

        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):

        """
        The moveit_hard_play_planned_path_from_file function performs the certain attempts to make moveit_play_Planned_path_from_file run successfully.
        Parameters : Path of the .yaml file to be played
                     Name of the .yaml file to be executed
                     Maximum number of attempts to be execute moveit_play_planned_path_from_file function
        Returning type : Returns flag_success as True on successfully playing the required trajectory
        """

        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()
        
        return True


    def add_box(self,a,b,c, timeout=4):

        """
        The add_box function is used to add the box at specific position in rviz scene to avoid the collision. 
        Parameters : Co-ordinates of the positions at which the box is to be added in rviz scene
                     Maximum time to wait for state update
        """

        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self._box_name
        scene = self._scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 0
        box_pose.pose.position.x = a    
        box_pose.pose.position.y = b     
        box_pose.pose.position.z = c
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

       
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self._box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

        """
        The wait_for_state_update function is to wait for some time until the state get updated.
        Parameters : Maximum time to wait for state update
        Returning type : Returns state after updation
        """

        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self._box_name
        scene = self._scene

        
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def attach_box(self, timeout=4):

        """
        The attach_box function is used to attach the box to the ur5 arm in rviz planning scene.
        Parameters : Maximum time to wait for state update
        """

        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self._box_name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names

        ## BEGIN_ attach_object
        ##
        ## Attaching Objects to the arm
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = 'manipulator'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)
    
    def detach_box(self, timeout=4):

        """
        The detach_box function is used to detach the box from the ur5 arm in rviz planning scene.
        Parameters : Maximum time to wait for state update
        """

        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self._box_name
        scene = self._scene
        eef_link = self._eef_link

        ## BEGIN_ detach_object
        ##
        ## Detaching Objects from the arm
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        """
        The remove_box function is used to remove the box which was added earlier in add_box function from rviz planning scene.
        Parameters : Maximum time to wait for state update
        """

        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self._box_name
        scene = self._scene

        ## BEGIN_remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
        

    def activate_gripper(self,flag):
        """
        The activate_gripper function is called when we need to activate the vaccum gripper of ur5 arm to pick a box.
        Parameters : flag as True to activate vaccum gripper 
        """

        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        
        try:
            activate_vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
            
            resp1 = activate_vacuum_gripper(flag)
            
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)  

    def deactivate_gripper(self,flag):
        """
        The deactivate_gripper function is called when we need to deactivate the vaccum gripper of ur5 arm to drop the box.
        Parameters : flag as False to deactivate vaccum gripper
        """
        
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        
        try:
            activate_vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
            
            resp1 = activate_vacuum_gripper(flag)
            
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)  

    def activate_conveyer(self,val):
        """
        The activate_conveyer function is used to control the speed of conveyor belt.
        Parameters : val is the power at which conveyor should be operated
        """

        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        
        try:
            activate_conveyer_belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            
            resp1 = activate_conveyer_belt(val)
            print("Conveyer starts")
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)    

    def return_pkg_name(self,x,y):
        """
        The return_pkg_name function returns package name based on its position in output image of opencv.
        Parameters : x and y co-ordinates of a package
        Returning type : Name of the package present at the provided position
        """

        ref_dict = {"packagen00": [128,315],"packagen01":[315,315],"packagen02":[502,315],"packagen10":[128,496],"packagen11":[315,496],"packagen12":[502,496],"packagen20":[128,643],"packagen21": [315,643],"packagen22":[502,643],"packagen30":[128,796],"packagen31":[315,796],"packagen32":[502,796],}
        for key,value in ref_dict.items():
              
              if ((x >= int(value[0])-10) and (x <= int(value[0])+10)) and ((y >= int(value[1])-10) and (y <= int(value[1])+10)) :
                print(type(value[0]))
                 
                return key  


    def get_dominant_colour(self, arg_img):
        """
        The get_dominant_colour function performs the work of detecting the color of all the packages present on the shelf.
        It tooks the feed from the 2d camera and process the image using opencv and detect the colour of the packages and store all information in the dictionary.
        From this function we are also calling update_inv_sheet and incoming_order functions.
        Parameters : arg_img as raw image output of 2D camera
        """

        global color_dict
        #global color_dict1
        global pkg_det
        color_dict = {}
        #color_dict1={}
        global LastPackageName
        LastPackageName = {"default": "default"}
        
        global ref_dict
        global order_list
        global counter
        global new_lst
        #global lst
        new_lst = []
        counter = {"i":0}
        order_list = []
        ref_dict = {"packagen00": [128,315],"packagen01":[315,315],"packagen02":[502,315],"packagen10":[128,496],"packagen11":[315,496],"packagen12":[502,496],"packagen20":[128,643],"packagen21": [315,643],"packagen22":[502,643],"packagen30":[128,796],"packagen31":[315,796],"packagen32":[502,796],}
        imgHSV = cv2.cvtColor(arg_img,cv2.COLOR_BGR2HSV)
         
        kernal = np.ones((5, 5), "uint8")

        red_lower = np.array([0,9,97])
        red_upper = np.array([0,255,102])
        red_mask = cv2.inRange(imgHSV,red_lower,red_upper)

        
        imgRed = cv2.bitwise_and(arg_img,arg_img,mask = red_mask)

          # Set range for green color and  
          # define mask 
        green_lower = np.array([59,117,87])
        green_upper = np.array([63,255,102])
        
        green_mask = cv2.inRange(imgHSV,green_lower,green_upper)

        
        imgGreen = cv2.bitwise_and(arg_img,arg_img,mask = green_mask)

          # Set range for yellow color and  
          # define mask 
        yellow_lower = np.array([29,16,97])
        yellow_upper = np.array([31,255,102])
        yellow_mask = cv2.inRange(imgHSV,yellow_lower,yellow_upper)

        
        imgYellow = cv2.bitwise_and(arg_img,arg_img,mask = yellow_mask)

        
        # Creating contour to track red color 
        image, contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            print("RED area --->"+ str(area))
            if(area > 300): 
                
                x, y, w, h = cv2.boundingRect(contour) 
                
                pkg_name = self.return_pkg_name(x,y)
                color_dict.update({str(pkg_name) : "RED" })
                print(x ,y,w,h)
                print("for red pos")
                arg_img = cv2.rectangle(arg_img, (x, y),(x + w, y + h),(0, 0, 255), 2) 
                  
                cv2.putText(arg_img, "Red Box", (x, y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 0, 255))   
                print(color_dict)  
        
        # Creating contour to track green color 
        image, contours, hierarchy = cv2.findContours(green_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) 
        
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour)
                pkg_name = self.return_pkg_name(x,y)
                color_dict.update({str(pkg_name) : "GREEN" })
                
                print("for green pos")
                print(x,y,w,h)
                arg_img = cv2.rectangle(arg_img, (x, y),(x + w, y + h),(0, 255, 0), 2) 
                  
                cv2.putText(arg_img, "Green Box", (x, y),cv2.FONT_HERSHEY_SIMPLEX,1.0, (0, 255, 0)) 
          

        # Creating contour to track yellow  color 
        image, contours, hierarchy = cv2.findContours(yellow_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) 
        
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour)
                pkg_name = self.return_pkg_name(x,y)
                color_dict.update({str(pkg_name) : "YELLOW" })
                
                print("for yellow pos")
                print(x,y,w,h)
                arg_img = cv2.rectangle(arg_img, (x, y),(x + w, y + h),(0, 255, 255), 2) 
                  
                cv2.putText(arg_img, "Yellow Box", (x, y),cv2.FONT_HERSHEY_SIMPLEX,1.0, (0, 255, 255)) 
          
        print(color_dict)
        print(len(color_dict))
        resized_red = cv2.resize(imgRed, (720/2, 1280/2))
        resized_green = cv2.resize(imgGreen, (720/2, 1280/2))
        resized_yellow = cv2.resize(imgYellow, (720/2, 1280/2))
        resized_original = cv2.resize(arg_img, (720/2, 1280/2))
        
        cv2.imshow("Multiple Color Detection of packages", resized_original) 
        cv2.waitKey(1)
        self.unsubscribe()

        # lst=sorted(color_dict)
        # print(lst)
        # for i in lst:
        #     for key,value  in color_dict.items():
        #         if i==key:
        #             color_dict1.update({str(key) : str(value) })
        #             print(color_dict1)
        #     #color_dict1[i]=(color_dict[j])
        #print("$$$$$$$$")
        #print(color_dict)
        # tinv = threading.Thread(target=self.update_inv_sheet, args=[])
        # tor = threading.Thread(target=self.incoming_order, args=[])
        # tinv.start()
        # tor.start()
        self.update_inv_sheet()
        self.incoming_order()
        print("I am printing list")
        print(order_list)
        print("Zalo part print")
        #rospy.Subscriber("/eyrc/vb/RpAaSgSa/orders", myMessage, func_callback_topic_my_topic)

    # 3. spin() simply keeps python from exiting until this node is stopped
        rospy.spin()




    def update_inv_sheet(self):
        """
        In update_inv_sheet function we are creating a dictionary for each package which is detected by image procesing and after creating the dictionary we are publishing dictionary on rostoic to push all the values in the inventory management spreadsheet.
        Parameters : None
        """

        pkg_det = {}
        pkg_det.update({str("Team Id") : "VB#1025" })
        pkg_det.update({str("Unique Id") : "RpAaSgSa" })
        pkg_det.update({str("Quantity") : "1" })
        pkg_det.update({str("id") : "Inventory" })
        var_handle_pub = rospy.Publisher('my_topic', myMessage, queue_size=10)
        var_loop_rate = rospy.Rate(1)
        date = str(self.get_time_str())
        for key,value in color_dict.items() :
            op="R"+str(key[8:9])+" C"+str(key[9:])
            pkg_det.update({str("Storage Number") : op })
            if value == "RED" :
                pkg_det.update({str("Item") : "Medicine" })
                pkg_det.update({str("Priority") : "HP" })
                pkg_det.update({str("Cost") : "300" })
                s1 = str(value[0])
                s2 = str(key[8:])
                s3 = str(date[2:4])
                s4 = str(date[5:7])
                s5 = s1+s2+s4+s3
                print(s5)
                pkg_det.update({str("SKU") : s5 })

            elif value == "YELLOW" :
                pkg_det.update({str("Item") : "Food" })
                pkg_det.update({str("Priority") : "MP" })
                pkg_det.update({str("Cost") : "200" })
                s1 = str(value[0])
                s2 = str(key[8:])
                s3 = str(date[2:4])
                s4 = str(date[5:7])
                s5 = s1+s2+s4+s3
                print(s5)
                pkg_det.update({str("SKU") : s5 })

            elif value == "GREEN" :
                pkg_det.update({str("Item") : "Clothes" })
                pkg_det.update({str("Priority") : "LP" })
                pkg_det.update({str("Cost") : "100" })
                s1 = str(value[0])
                s2 = str(key[8:])
                s3 = str(date[2:4])
                s4 = str(date[5:7])
                s5 = s1+s2+s4+s3
                print(s5)
                pkg_det.update({str("SKU") : s5 })

            else :
                pass
            #print("aaaa  "+str(pkg_det))
            obj_msg = myMessage()
            #pkg_dict = message_converter.convert_dictionary_to_ros_message('std_msgs/String', pkg_det)
            #pkg_dict = json.dumps(pkg_det, separators=(',', ':') , sort_keys=True)
            pkg_dit = str(pkg_det)
            print("1111")
            print(pkg_dit)
            obj_msg.pkg_dit = pkg_dit
            rospy.sleep(1)
            rospy.loginfo("Publishing: ")
            #rospy.loginfo(obj_msg)

            var_handle_pub.publish(obj_msg)

            var_loop_rate.sleep()
            rospy.sleep(1)





    def incoming_order(self):
        """
        In incoming_order function we are subscribing /eyrc/vb/RpAaSgSa/orders topic to get the incoming orders.
        Parameters : None
        """

        broker_url = "broker.mqttdashboard.com"
        broker_port = 1883
        
        print("1")
        try:
            mqtt_client = mqtt.Client()
            print("11")
            mqtt_client.on_message = self.mqtt_sub_callback
            mqtt_client.connect(broker_url, broker_port)
            print("111")
            mqtt_client.subscribe("/eyrc/vb/RpAaSgSa/orders", qos=0)
            time.sleep(1) # wait
            print("1111")
            # mqtt_client.loop_forever() # starts a blocking infinite loop
            mqtt_client.loop_start()    # starts a new thread
            return 0
        except:
            #return -1
            print("-111")

    def mqtt_sub_callback(self, client, userdata, message):
        """
        This function is called whenever new order is recieved via ros-iot bridge.
        pkg_det is dictionary containing all order details which is used for publishing on mqtt topic that is used to update spreadsheet.
        """
        counter["i"] = counter["i"] + 1
        payload = str(message.payload.decode("utf-8"))
        print("11111")
        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)
        print("111111")
        
        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload
        print(payload)
        payload = eval(payload)
        print(type(payload))

        pkg_det = {}
        pkg_det.update({str("Team Id") : "VB#1025" })
        pkg_det.update({str("Unique Id") : "RpAaSgSa" })
        pkg_det.update({str("Order Quantity") : payload["qty"] })
        pkg_det.update({str("id") : "IncomingOrders" })
        pkg_det.update({str("Order ID") : payload["order_id"] })
        pkg_det.update({str("Order Date and Time") : payload["order_time"] })
        pkg_det.update({str("Item") : payload["item"] })

        order_list.append([payload["item"],payload["order_id"],payload["city"]])
        new_lst.append(payload["item"])
        print(order_list)
        print(new_lst)

        pkg_det.update({str("City") : payload["city"] })
        pkg_det.update({str("Longitude") : payload["lon"] })
        pkg_det.update({str("Latitude") : payload["lat"] })
        var_handle_pub = rospy.Publisher('my_topic', myMessage, queue_size=10)
        var_loop_rate = rospy.Rate(1)

        if payload["item"] == "Medicine" :
            #pkg_det.update({str("Items") : "Medicine" })
            pkg_det.update({str("Priority") : "HP" })
            pkg_det.update({str("Cost") : "300" })

        elif payload["item"] == "Food" :
            #pkg_det.update({str("Items") : "Food" })

            pkg_det.update({str("Priority") : "MP" })
            pkg_det.update({str("Cost") : "200" })

        elif payload["item"] == "Clothes" :
            #pkg_det.update({str("Items") : "Clothes" })
            pkg_det.update({str("Priority") : "LP" })
            pkg_det.update({str("Cost") : "100" })

        else :
            pass
        #self._handle_ros_pub.publish(msg_mqtt_sub)
        #print("111111")
        
        obj_msg = myMessage()
        pkg_dit = str(pkg_det)
        print("1111")
        print(pkg_dit)
        obj_msg.pkg_dit = pkg_dit
        rospy.sleep(1)
        rospy.loginfo("Publishing: ")
        #rospy.loginfo(obj_msg)

        # var_handle_pub.publish(obj_msg)

        # var_loop_rate.sleep()

        
        if self.flagg == "True":
            self.flagg = "False"
            print(self.flagg)
            tpub = threading.Thread(target=var_handle_pub.publish, args=[obj_msg])
            tpic = threading.Thread(target=self.pick_logic, args=[])
            tpub.start()
            var_loop_rate.sleep()
            #rospy.sleep(1)
            tpic.start()

        else :
            tpub = threading.Thread(target=var_handle_pub.publish, args=[obj_msg])
            tpub.start()
            var_loop_rate.sleep()

        #self.pick_logic()
        # if counter["i"] == 9 :
        #     #self.unsubscribe()
        #     #mqtt_client.unsubscribe("/eyrc/vb/RpAaSgSa/orders")
        #     print("Zal baba unsubscribe")


    def callback(self,data):

        """
        The callback function is the callback function of the 2d camera in which we store and use all the output values of the 2d camera.
        """
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          rospy.logerr(e)

        (rows,cols,channels) = cv_image.shape
        
        image = cv_image

        # Resize a 720x1280 image to 360x640 to fit it on the screen
        resized_image = cv2.resize(image, (720/2, 1280/2)) 

        cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)

        rospy.loginfo(self.get_dominant_colour(image))

        cv2.waitKey(3)


    def pick_logic(self) :
        """
        This functions apllies logic to pick the packages from shelf according to priorities of pending orders and decides package to be picked. Then calls select_trajec function to play respwctive trajectory saved for that package.
        Parameters : None 
        """
        #color_dict = sorted(color_dict)
        while True:
            if len(order_list) != 0 :
                break

        
        # for lst1 in order_list :
        #     new_lst.append(lst1[0])
        #     print(new_lst)
        #for x in order_list :
        #i = -1
        #for y in new_lst :
        lst = []
        lst=sorted(color_dict.keys())
        lst.reverse()
        if "Medicine" in new_lst :
            i = new_lst.index("Medicine")
            for n in lst :
                for key,value in color_dict.items() :
                    # print(" key in color_dict" +str(key))
                    # print(n)
                    if n == key  and value == "RED":
                    #if value == "RED" :
                        pkg_nav = key

                        lst.remove(n)

        elif  "Food" in new_lst :
            i = new_lst.index("Food")
            for n in lst :
                for key,value in color_dict.items() :
                    # print(" key in color_dict" +str(key))
                    # print(n)
                    if n == key and value == "YELLOW" :
                        pkg_nav = key
                        lst.remove(n)

        elif "Clothes" in new_lst:
            i = new_lst.index("Clothes")
            for n in lst :
                for key,value in color_dict.items() :
                    #print(" key in color_dict" +str(key))
                    # print(n)
                    if n == key and value == "GREEN" :
                        pkg_nav = key
                        lst.remove(n)

        else :
            pass

        ord_lst = order_list[i]
        print("@@@@@@")
        print(ord_lst)
        print("********")
        print(order_list)
        print(new_lst)
        print(color_dict)
        del new_lst[i]
        del order_list[i]
        del color_dict[pkg_nav]
        print(order_list)
        print(new_lst)
        print(color_dict)
        self.select_trajec(pkg_nav,ord_lst)



        ##### for x in order_list:

        #     if "Medicine" in x :
        #         for key,value in color_dict1.items() :
        #             if value == "RED" :
        #                 pkg_nav = key
        #                 ord_lst = x 




        #     elif "Food" in x :
        #         for key,value in color_dict1.items() :
        #             if value == "YELLOW" :
        #                 pkg_nav = key
        #                 ord_lst = x 

        #     elif "Clothes" in x :
        #         for key,value in color_dict1.items() :
        #             if value == "GREEN" :
        #                 pkg_nav = key
        #                 ord_lst = x 

        #     else :
        #         pass 

            
            
            
            #order_list.remove(x)
            
    def select_trajec(self,pkg_nav,ord_lst) :

        """
        The select_trajec function is selecting the trajectories according to the package name and accordingly playing that saved trajectory to pick and place that particular package.
        Parameters : Name of package to be picked from shelf
                     List containing item, order Id and city of the order for which package is being picked 
        """

        if pkg_nav == "packagen00":
            a = 0.28
            b = -0.41
            c = 1.91
            flag = True
            rospy.logwarn("1. Playing allzeros to package00 (red) Trajectory File")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg00_pick.yaml', 5)
            
            t1 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
            t2 = threading.Thread(target=ur5.attach_box, args=[])
            t3 = threading.Thread(target=ur5.activate_gripper, args=[flag])
            t1.start() 
            t2.start()
            t3.start()
            

            flag = False
            rospy.logwarn("2. Playing package00 to the home pose  Trajectory File to place package")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg00_place.yaml', 5)
            
            t4 = threading.Thread(target=ur5.detach_box, args=[]) 
            t5 = threading.Thread(target=ur5.remove_box, args=[])
            # thread to deactivate the Vaccum Gripper in Gazebo
            t6 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

            t4.start() 
            t5.start()
            t6.start()
            #self.flagg = "True"
            print(self.flagg)
            tsend = threading.Thread(target=self.update_disp, args=[ord_lst]) 
            tpick = threading.Thread(target=self.pick_logic, args=[])
            #self.pick_logic()
            tsend.start()
            tpick.start()

        elif pkg_nav == "packagen01":
            # saved trajectory play for package01
            a = 0
            b = -0.41
            c = 1.91
            flag = True
            rospy.logwarn("3. Playing home_pose to package01(green) Trajectory File")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '2_home_to_pkg01_green1.yaml', 5)
            
            t7 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
            t8 = threading.Thread(target=ur5.attach_box, args=[])
            t9 = threading.Thread(target=ur5.activate_gripper, args=[flag])
            t7.start() 
            t8.start()
            t9.start()
            

            flag = False
            rospy.logwarn("4. Playing package01 to the home pose  Trajectory File to place package")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '2_Pkg01_green_place1.yaml', 5)
            
            t10 = threading.Thread(target=ur5.detach_box, args=[]) 
            t11 = threading.Thread(target=ur5.remove_box, args=[])
            # thread to deactivate the Vaccum Gripper in Gazebo
            t12 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

            t10.start() 
            t11.start()
            t12.start()
            #self.flagg = "True"
            print(self.flagg)
            #self.pick_logic()
            tsend = threading.Thread(target=self.update_disp, args=[ord_lst]) 
            tpick = threading.Thread(target=self.pick_logic, args=[])
            #self.pick_logic()
            tsend.start()
            tpick.start()

        elif pkg_nav == "packagen02":

            # saved trajectory play for package02
            a = -0.28
            b = -0.41
            c = 1.91
            flag = True
            rospy.logwarn("5. Playing home_pose to package02 (yellow) Trajectory File")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '3_home_to_pkg02_yellow.yaml', 5)
            
            t13 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
            t14 = threading.Thread(target=ur5.attach_box, args=[])
            t15 = threading.Thread(target=ur5.activate_gripper, args=[flag])
            t13.start() 
            t14.start()
            t15.start()
            

            flag = False
            rospy.logwarn("6.  Playing package02 to the home pose  Trajectory File to place package")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '3_Pkg02_yellow_place.yaml', 5)
            
            t16 = threading.Thread(target=ur5.detach_box, args=[]) 
            t17= threading.Thread(target=ur5.remove_box, args=[])
            # thread to deactivate the Vaccum Gripper in Gazebo
            t18 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

            t16.start() 
            t17.start()
            t18.start()
            #self.flagg = "True"
            print(self.flagg)
            #self.pick_logic()
            tsend = threading.Thread(target=self.update_disp, args=[ord_lst]) 
            tpick = threading.Thread(target=self.pick_logic, args=[])
            #self.pick_logic()
            tsend.start()
            tpick.start()

        elif pkg_nav == "packagen10":

            # saved trajectory play for package10
            a = 0.28
            b = -0.41
            c = 1.64
            flag = True
            rospy.logwarn("7. Playing home_pose to package10 (green) Trajectory File")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '4_home_to_pkg10_green.yaml', 5)
            
            t19 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
            t20 = threading.Thread(target=ur5.attach_box, args=[])
            t21= threading.Thread(target=ur5.activate_gripper, args=[flag])
            t19.start() 
            t20.start()
            t21.start()
            

            flag = False
            rospy.logwarn("8. Playing package10 to the home pose  Trajectory File to place package")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '4_Pkg10_green_place.yaml', 5)
            
            t22 = threading.Thread(target=ur5.detach_box, args=[]) 
            t23 = threading.Thread(target=ur5.remove_box, args=[])
            # thread to deactivate the Vaccum Gripper in Gazebo
            t24 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

            t22.start() 
            t23.start()
            t24.start()
            #self.flagg = "True"
            print(self.flagg)
            #self.pick_logic()
            tsend = threading.Thread(target=self.update_disp, args=[ord_lst]) 
            tpick = threading.Thread(target=self.pick_logic, args=[])
            #self.pick_logic()
            tsend.start()
            tpick.start()

        elif pkg_nav == "packagen11":

            # saved trajectory play for package11
            a = 0
            b = -0.41
            c = 1.64
            flag = True
            rospy.logwarn("9. Playing home_pose to package11 (yellow) Trajectory File")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '5_home_to_pkg11_yellow.yaml', 5)
            
            t25 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
            t26 = threading.Thread(target=ur5.attach_box, args=[])
            t27 = threading.Thread(target=ur5.activate_gripper, args=[flag])
            t25.start() 
            t26.start()
            t27.start()
            

            flag = False
            rospy.logwarn("10. Playing package11 to the home pose  Trajectory File to place package")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '5_Pkg1_yellow_place.yaml', 5)
            
            t28 = threading.Thread(target=ur5.detach_box, args=[]) 
            t29 = threading.Thread(target=ur5.remove_box, args=[])
            # thread to deactivate the Vaccum Gripper in Gazebo
            t30 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

            t28.start() 
            t29.start()
            t30.start()
            #self.flagg = "True"
            print(self.flagg)
            #self.pick_logic()
            tsend = threading.Thread(target=self.update_disp, args=[ord_lst]) 
            tpick = threading.Thread(target=self.pick_logic, args=[])
            #self.pick_logic()
            tsend.start()
            tpick.start()

        elif pkg_nav == "packagen12":

             # saved trajectory play for package12
            a = -0.28
            b = -0.41
            c = 1.64
            flag = True
            rospy.logwarn("11. Playing home_pose to package12 (green) Trajectory File")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '6_home_to_pkg12_green1.yaml', 5)
            
            t31 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
            t32 = threading.Thread(target=ur5.attach_box, args=[])
            t33 = threading.Thread(target=ur5.activate_gripper, args=[flag])
            t31.start() 
            t32.start()
            t33.start()
            

            flag = False
            rospy.logwarn("12. Playing package12 to the home pose  Trajectory File to place package")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '6_Pkg12_red_place1.yaml', 5)
            
            t34 = threading.Thread(target=ur5.detach_box, args=[]) 
            t35 = threading.Thread(target=ur5.remove_box, args=[])
            # thread to deactivate the Vaccum Gripper in Gazebo
            t36 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

            t34.start() 
            t35.start()
            t36.start()
            #self.flagg = "True"
            print(self.flagg)
            #self.pick_logic()
            tsend = threading.Thread(target=self.update_disp, args=[ord_lst]) 
            tpick = threading.Thread(target=self.pick_logic, args=[])
            #self.pick_logic()
            tsend.start()
            tpick.start()

        elif pkg_nav == "packagen20":
                
            a = 0.28
            b = -0.41
            c = 1.42
            flag = True
            rospy.logwarn("13. Playing home_pose to package20  Trajectory File")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '7_home_to_pkg20.yaml', 5)
            
            t37 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
            t38 = threading.Thread(target=ur5.attach_box, args=[])
            t39 = threading.Thread(target=ur5.activate_gripper, args=[flag])
            t37.start() 
            t38.start()
            t39.start()
            

            flag = False
            rospy.logwarn("14. Playing package20 to the home pose  Trajectory File to place package")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '7_pkg20_to_home_place.yaml', 5)
            
            t40 = threading.Thread(target=ur5.detach_box, args=[]) 
            t41 = threading.Thread(target=ur5.remove_box, args=[])
            # thread to deactivate the Vaccum Gripper in Gazebo
            t42 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

            t40.start() 
            t41.start()
            t42.start()
            #self.flagg = "True"
            print(self.flagg)
            #self.pick_logic()
            tsend = threading.Thread(target=self.update_disp, args=[ord_lst]) 
            tpick = threading.Thread(target=self.pick_logic, args=[])
            #self.pick_logic()
            tsend.start()
            tpick.start()


        elif pkg_nav == "packagen21":

            #  # saved trajectory play for package21
            a = 0
            b = -0.41
            c = 1.42
            flag = True
            rospy.logwarn("15. Playing home_pose to package21 (red) Trajectory File")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '8_home_to_pkg21_red.yaml', 5)
            
            t37 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
            t38 = threading.Thread(target=ur5.attach_box, args=[])
            t39 = threading.Thread(target=ur5.activate_gripper, args=[flag])
            t37.start() 
            t38.start()
            t39.start()
            

            flag = False
            rospy.logwarn("16. Playing package21 to the home pose  Trajectory File to place package")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '8_Pkg21_red_place.yaml', 5)
            
            t40 = threading.Thread(target=ur5.detach_box, args=[]) 
            t41 = threading.Thread(target=ur5.remove_box, args=[])
            # thread to deactivate the Vaccum Gripper in Gazebo
            t42 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

            t40.start() 
            t41.start()
            t42.start()
            #self.flagg = "True"
            print(self.flagg)
            #self.pick_logic()
            tsend = threading.Thread(target=self.update_disp, args=[ord_lst]) 
            tpick = threading.Thread(target=self.pick_logic, args=[])
            #self.pick_logic()
            tsend.start()
            tpick.start()

        elif pkg_nav == "packagen22":
            #  # saved trajectory play for package22
            a = -0.28
            b = -0.41
            c = 1.42
            flag = True
            rospy.logwarn("17. Playing home_pose to package22 (green) Trajectory File")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '9_home_to_pkg22_green1.yaml', 5)
            
            t43 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
            t44 = threading.Thread(target=ur5.attach_box, args=[])
            t45 = threading.Thread(target=ur5.activate_gripper, args=[flag])
            t43.start() 
            t44.start()
            t45.start()
            

            flag = False
            rospy.logwarn("18. Playing package22 to the home pose  Trajectory File to place package")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '9_Pkg22_green_place1.yaml', 5)
            
            t46 = threading.Thread(target=ur5.detach_box, args=[]) 
            t47 = threading.Thread(target=ur5.remove_box, args=[])
            # thread to deactivate the Vaccum Gripper in Gazebo
            t48 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

            t46.start() 
            t47.start()
            t48.start()
            #self.flagg = "True"
            print(self.flagg)
            #self.pick_logic()
            tsend = threading.Thread(target=self.update_disp, args=[ord_lst]) 
            tpick = threading.Thread(target=self.pick_logic, args=[])
            #self.pick_logic()
            tsend.start()
            tpick.start()

        elif pkg_nav == "packagen30":

             # saved trajectory play for package30
            a = 0.28
            b = -0.41
            c = 1.42
            flag = True
            rospy.logwarn("13. Playing home_pose to package30  Trajectory File")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '10home_to_pkg30.yaml', 5)
            rospy.sleep(1)
            t49 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
            t50 = threading.Thread(target=ur5.attach_box, args=[])
            t51 = threading.Thread(target=ur5.activate_gripper, args=[flag])
            t49.start() 
            t50.start()
            t51.start()
            

            flag = False
            rospy.logwarn("14. Playing package30 to the home pose  Trajectory File to place package")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '10pkg30_to_home.yaml', 5)
            rospy.sleep(1)
            t52 = threading.Thread(target=ur5.detach_box, args=[]) 
            t53 = threading.Thread(target=ur5.remove_box, args=[])
            # thread to deactivate the Vaccum Gripper in Gazebo
            t54 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

            t52.start() 
            t53.start()
            t54.start()
            #self.flagg = "True"
            print(self.flagg)
            #self.pick_logic()
            tsend = threading.Thread(target=self.update_disp, args=[ord_lst]) 
            tpick = threading.Thread(target=self.pick_logic, args=[])
            #self.pick_logic()
            tsend.start()
            tpick.start()
            

        elif pkg_nav == "packagen31":
            a = 0
            b = -0.41
            c = 1.19
            flag = True
            rospy.logwarn("13. Playing home_pose to package30  Trajectory File")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg3_1_pick.yaml', 5)
            rospy.sleep(1)
            t49 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
            t50 = threading.Thread(target=ur5.attach_box, args=[])
            t51 = threading.Thread(target=ur5.activate_gripper, args=[flag])
            t49.start() 
            t50.start()
            t51.start()
            

            flag = False
            rospy.logwarn("14. Playing package30 to the home pose  Trajectory File to place package")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg3_1_place.yaml', 5)
            rospy.sleep(1)
            t52 = threading.Thread(target=ur5.detach_box, args=[]) 
            t53 = threading.Thread(target=ur5.remove_box, args=[])
            # thread to deactivate the Vaccum Gripper in Gazebo
            t54 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

            t52.start() 
            t53.start()
            t54.start()
            #self.flagg = "True"
            print(self.flagg)
            #self.pick_logic()
            tsend = threading.Thread(target=self.update_disp, args=[ord_lst]) 
            tpick = threading.Thread(target=self.pick_logic, args=[])
            #self.pick_logic()
            tsend.start()
            self.pick_logic()

        elif pkg_nav == "packagen32":
            a = -0.28
            b = -0.41
            c = 1.19
            flag = True
            rospy.logwarn("13. Playing home_pose to package30  Trajectory File")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg3_2_pick.yaml', 5)
            rospy.sleep(1)
            t49 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
            t50 = threading.Thread(target=ur5.attach_box, args=[])
            t51 = threading.Thread(target=ur5.activate_gripper, args=[flag])
            t49.start() 
            t50.start()
            t51.start()
            

            flag = False
            rospy.logwarn("14. Playing package30 to the home pose  Trajectory File to place package")
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg3_2_place.yaml', 5)
            rospy.sleep(1)
            t52 = threading.Thread(target=ur5.detach_box, args=[]) 
            t53 = threading.Thread(target=ur5.remove_box, args=[])
            # thread to deactivate the Vaccum Gripper in Gazebo
            t54 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

            t52.start() 
            t53.start()
            t54.start()
            #self.flagg = "True"
            print(self.flagg)
            #self.pick_logic()
            tsend = threading.Thread(target=self.update_disp, args=[ord_lst]) 
            tpick = threading.Thread(target=self.pick_logic, args=[])
            #self.pick_logic()
            tsend.start()
            self.pick_logic()

        else :
            self.pick_logic()





    def get_time_str(self):

        """
        The get_time_str function returns the current date and time in year-month-date hour:minutes:sec this format.
        Parameters : None
        Returning type : Returns string containing date and time
        """

        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%Y-%m-%d %H:%M:%S')

        return str_time

    def update_disp(self,ord_lst):

        """
        In update_disp function we are creating a dictionary for each dispatched package (package which is placed on conveyor by ur5_1 arm) and after creating the dictionary we are publishing dictionary on rostoic to puch all the values in the ordersshipped spreadsheet and we are also publishing some values on topic named your_topic for making those values accessable in other python node.
        Parameters : A list conating item, order Id and city of the order for which respective package is dispatched
        """
        pkg_disp = {}
        var_handle_pub = rospy.Publisher('dispatch_topic', myMessage, queue_size=10)
        var_loop_rate = rospy.Rate(1)

        var_handle_pub_2 = rospy.Publisher('your_topic', myMessage, queue_size=10)
        var_loop_rate = rospy.Rate(1)

        pkg_disp.update({str("id") : "OrdersDispatched" })
        pkg_disp.update({str("Team Id") : "VB#1025"})
        pkg_disp.update({str("Unique Id") : "RpAaSgSa"})
        pkg_disp.update({str("Order ID") : ord_lst[1]})
        pkg_disp.update({str("City") : ord_lst[2] })
        pkg_disp.update({str("Item") : ord_lst[0] })
        date = self.get_time_str()
        print(date)
        pkg_disp.update({str("Dispatch Date and Time") : date })
        pkg_disp.update({str("Dispatch Quantity") : "1" })
        pkg_disp.update({str("Dispatch Status") : "YES" })


        if ord_lst[0] == "Medicine" :
            #pkg_det.update({str("Items") : "Medicine" })
            pkg_disp.update({str("Priority") : "HP" })
            pkg_disp.update({str("Cost") : "300" })

        elif ord_lst[0] == "Food" :
            #pkg_det.update({str("Items") : "Food" })

            pkg_disp.update({str("Priority") : "MP" })
            pkg_disp.update({str("Cost") : "200" })

        elif ord_lst[0] == "Clothes" :
            #pkg_det.update({str("Items") : "Clothes" })
            pkg_disp.update({str("Priority") : "LP" })
            pkg_disp.update({str("Cost") : "100" })

        else :
            pass

        obj_msg = myMessage()

        #pkg_dict = message_converter.convert_dictionary_to_ros_message('std_msgs/String', pkg_det)
        #pkg_dict = json.dumps(pkg_det, separators=(',', ':') , sort_keys=True)
        pkg_dit = str(pkg_disp)
        print("1111")
        print(pkg_dit)
        obj_msg.pkg_dit = pkg_dit
        rospy.sleep(1)
        rospy.loginfo("Publishing:in  Dispatch sheet ")
        #rospy.loginfo(obj_msg)

        var_handle_pub.publish(obj_msg)
        var_loop_rate.sleep()
        #rospy.sleep(1)

        obj_msg_2 = myMessage()
        #pkg_dict = message_converter.convert_dictionary_to_ros_message('std_msgs/String', pkg_det)
        #pkg_dict = json.dumps(pkg_det, separators=(',', ':') , sort_keys=True)
        pkg_dit_2 = str(ord_lst)
        print("2222")
        print(pkg_dit_2)
        obj_msg_2.pkg_dit = pkg_dit_2
        rospy.sleep(1)
        rospy.loginfo("Publishing:in  pkg_col_det node sheet ")
        #rospy.loginfo(obj_msg)

        var_handle_pub_2.publish(obj_msg_2)

        var_loop_rate.sleep()
        #rospy.sleep(1)




    def unsubscribe(self):

        """
        This function is used to unsubscribe or unregister the rostoic of 2d camera.
        """

        # use the saved subscriber object to unregister the subscriber
        self.image_sub.unregister()
        print("successfully unsubscribe")



    # Destructor

    def __del__(self):

        """
        This is the destructor of the class.
        """

        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def func_callback_topic_my_topic(obj_msg):

    """
    The func_callback_topic_my_topic is the callback function of 
    """

    # rospy.loginfo("Data Received: (%d, %d, %d )", obj_msg.final_x,
    #               obj_msg.final_y, obj_msg.final_theta)
    # goal_handle['x']=obj_msg.final_x
    # goal_handle['y']=obj_msg.final_y
    # goal_handle['theta']=obj_msg.final_theta
    # action_server.on_goal(goal_handle)
    print(type(obj_msg.pkg_dit))

    print(obj_msg.pkg_dit)

def main(args):
    rospy.sleep(145)
    global ur5
    ur5 = Ur5Moveit("ur5_1")

    lst_joint_angles_home_place = [math.radians(172),
                          math.radians(-46),
                          math.radians(59),
                          math.radians(-109),
                          math.radians(-90),
                          math.radians(0)]
    ur5.set_joint_angles(lst_joint_angles_home_place)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
  
    cv2.destroyAllWindows()

        

    del ur5



if __name__ == '__main__':
    #rospy.sleep(90)
    main(sys.argv)
