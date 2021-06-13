#!/usr/bin/env python
#https://www.youtube.com/watch?v=pGVOTsrb13E
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

import yaml
import os
import math
import time
import sys
import copy
import datetime, timedelta
import threading



from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


import cv2
import numpy as np
from pkg_task5.msg import myMessage
from pkg_vb_sim.srv import conveyorBeltPowerMsg 
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.msg import LogicalCameraImage
global list_of_list_2
list_of_list_2 = [] 
def empty(a):
  """
  This is just a empty function and it is called when we are trying to detect the color of packages.
  """
  pass

class Camera1:

  def __init__(self, arg_robot_name):


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
    self._pkg_path = rp.get_path('pkg_task4')
    self._file_path = self._pkg_path + '/config/saved_trajectories_shelf/'
    rospy.loginfo( "Package Path: {}".format(self._file_path) )


    rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')


    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)


  def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()
  def go_to_pose(self, arg_pose):

        """
        This function is used for ur5 arm to go to a specific position. 
        """

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan


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

  
                # self.clear_octomap()

  def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):

        """ Function moveit_play_planned_path_from_file is used to play the trajectories saved in config folder. """

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
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        
        try:
            activate_vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
            
            resp1 = activate_vacuum_gripper(flag)
            
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)  


  def deactivate_gripper(self,flag):
        """
        The deactivate_gripper function is called when we need to deactivate the vaccum gripper of ur5 arm to drop the box.
        Parameters : flag as False to deactivate vaccum gripper
        """
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        
        try:
            activate_vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
            
            resp1 = activate_vacuum_gripper(flag)
            
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
        Parameters : 
    """

    global color_dict
    color_dict = {}
    global LastPackageName
    LastPackageName = {"default": "default"}
    
    
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
    
   
    val2 = 100
    ic.activate_conveyer(val2)

    rospy.logwarn("18. Playing allzeros to the home pose  Trajectory File to pick package")
    ic.moveit_hard_play_planned_path_from_file(ic._file_path, 'allzeros_to_home_ur5_2.yaml', 5)

    # thread1 = threading.Thread(target=rospy.Subscriber, args=["/eyrc/vb/logical_camera_2", LogicalCameraImage, func_callback_topic_my_topic])
    # thread2 = threading.Thread(target=rospy.Subscriber, args=["your_topic", myMessage, func_callback_topic_your_topic])
    # thread1.start() 
    # #rospy.spin()
    # thread2.start()
    #rospy.spin()
    rospy.Subscriber("your_topic", myMessage, func_callback_topic_your_topic)
    rospy.sleep(5)
    rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, func_callback_topic_my_topic)
    
    # print("inside........")
    # rospy.spin()
    # print("outside.......")


    
  
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

  
  def sort_pkg(self,pkg_type,pose):
    """
    The sort_pkg function is the main function in this node as it is doing the job of sorting the packages according to the color.
    Parameters : Name of the package tobe sorted
                 Position of the package on conveyor
    """
    print(pkg_type)
    print(color_dict)
    print(len(color_dict))

    a = -0.80
    b = 0
    c = 0.99
    #logical camera2 is present at (-0.8,0,2) with respect to world frame

    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x = -0.80
    ur5_pose_2.position.y = -0.004541
    ur5_pose_2.position.z = (2-pose.position.x) + 0.20
    ur5_pose_2.orientation.x = -0.5
    ur5_pose_2.orientation.y = -0.5
    ur5_pose_2.orientation.z = 0.5
    ur5_pose_2.orientation.w = 0.5

    lst_joint_angles_home = [math.radians(172),
                          math.radians(-41),
                          math.radians(59),
                          math.radians(-109),
                          math.radians(-90),
                          math.radians(-8)]

    lst_joint_angles_red_bin = [math.radians(91),
                          math.radians(-70),
                          math.radians(62),
                          math.radians(-81),
                          math.radians(-90),
                          math.radians(-16)]

    
    lst_joint_angles_green_bin = [math.radians(90),
                          math.radians(-85),
                          math.radians(-98),
                          math.radians(-81),
                          math.radians(91),
                          math.radians(5)]

    lst_joint_angles_yellow_bin = [math.radians(-5),
                          math.radians(-74),
                          math.radians(62),
                          math.radians(-76),
                          math.radians(-88),
                          math.radians(-16)]


          
    if pose.position.y >= -0.1 and pose.position.y <= 0.1 and pkg_type != "ur5":

      val2 = 0
      ic.activate_conveyer(val2)

      
          
      for key,value in color_dict.items():
        if key == pkg_type:
          colour = value 
          print(colour)
          
           

          if colour == "RED" :
            
            print("Hi i am red ")
            
            ic.go_to_pose(ur5_pose_2)
            
            flag = True
            t4 = threading.Thread(target=ic.add_box, args=[a,b,c]) 
            t5 = threading.Thread(target=ic.attach_box, args=[])
            t6 = threading.Thread(target=ic.activate_gripper, args=[flag])
            t4.start() 
            t5.start()
            t6.start()


            
            rospy.sleep(1)
            
            val2 = 25
            
            tset = threading.Thread(target=ic.set_joint_angles, args=[lst_joint_angles_red_bin])
            tcon = threading.Thread(target=ic.activate_conveyer, args=[val2])
            
            tset.start()
            tcon.start()
            
            tset.join()
            tcon.join()

            flag = False
            
            t7 = threading.Thread(target=ic.detach_box, args=[]) 
            t8 = threading.Thread(target=ic.remove_box, args=[])
            # Thread to deactivate the Vaccum Gripper in Gazebo
            t9 = threading.Thread(target=ic.deactivate_gripper, args=[flag])

            t7.start() 
            t8.start()
            t9.start()

            print(list_of_list_2)
            val2 = 100
            ic.activate_conveyer(val2)
            print("**")
            ic.update_shipped()
            print("***")
            rospy.logwarn(". Playing red bin to the home pose  Trajectory File to pick package")
            ic.moveit_hard_play_planned_path_from_file(ic._file_path, 'red_bin_to_home_ur5_2.yaml', 5)
            


            del color_dict[key]
            
            LastPackageName["default"]=pkg_type
            
          elif colour == "GREEN":

            print("Hi I am GREEN pkg")
            flag = True
            ic.go_to_pose(ur5_pose_2)
            

            t4 = threading.Thread(target=ic.add_box, args=[a,b,c]) 
            t5 = threading.Thread(target=ic.attach_box, args=[])
            t6 = threading.Thread(target=ic.activate_gripper, args=[flag])
            t4.start() 
            t5.start()
            t6.start()
            
            rospy.sleep(1)
            val2 = 25
            tcon = threading.Thread(target=ic.activate_conveyer, args=[val2])
            tset = threading.Thread(target=ic.set_joint_angles, args=[lst_joint_angles_green_bin])
            
            tset.start()
            tcon.start()

            tset.join()
            tcon.join()

            flag = False
            
            t7 = threading.Thread(target=ic.detach_box, args=[]) 
            t8 = threading.Thread(target=ic.remove_box, args=[])
            # Thread to deactivate the Vaccum Gripper in Gazebo
            t9 = threading.Thread(target=ic.deactivate_gripper, args=[flag])

            t7.start() 
            t8.start()
            t9.start()
            
            print(list_of_list_2)
            val2 = 100
            ic.activate_conveyer(val2)
            print("**")
            ic.update_shipped()
            print("***")
            rospy.logwarn(". Playing green bin to the home pose  Trajectory File to pick package")
            ic.moveit_hard_play_planned_path_from_file(ic._file_path, 'green_bin_to_home_ur5_2.yaml', 5)
            

            del color_dict[key]
            
            LastPackageName["default"]=pkg_type

          elif colour == "YELLOW" :

            print("Hi I am yellow pkg")
            ic.go_to_pose(ur5_pose_2)
            
            flag = True
            t4 = threading.Thread(target=ic.add_box, args=[a,b,c]) 
            t5 = threading.Thread(target=ic.attach_box, args=[])
            t6 = threading.Thread(target=ic.activate_gripper, args=[flag])
            t4.start() 
            t5.start()
            t6.start()

            

            rospy.sleep(1)
            val2 = 25
            flag = False
            
            tcon = threading.Thread(target=ic.activate_conveyer, args=[val2])
            tset = threading.Thread(target=ic.set_joint_angles, args=[lst_joint_angles_yellow_bin])
            tset.start()
            tcon.start()
            tset.join()
            tcon.join()
            
            t7 = threading.Thread(target=ic.detach_box, args=[]) 
            t8 = threading.Thread(target=ic.remove_box, args=[])
            # Thread to deactivate the Vaccum Gripper in Gazebo
            t9 = threading.Thread(target=ic.deactivate_gripper, args=[flag])

            t7.start() 
            t8.start()
            t9.start()

            
            
            val2 = 100
            
            tcon = threading.Thread(target=ic.activate_conveyer, args=[val2])
            tset = threading.Thread(target=ic.set_joint_angles, args=[lst_joint_angles_home])
            tset.start()
            tcon.start()

            tset.join()
            tcon.join()
            print(list_of_list_2)
            print("**")
            ic.update_shipped()
            print("***")
            del color_dict[key]
            
            LastPackageName["default"]=pkg_type

          else :
            pass
    
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
    
  def update_shipped(self) :
    """
    In update_shipped function we are creating a dictionary for each shipped package and after creating the dictionary we are publishing dictionary on rostoic to puch all the values in the ordersshipped spreadsheet.
    Parameters : None
    """

    pkg_shipp = {}
    
    pkg_shipp.update({str("id") : "OrdersShipped"})
    pkg_shipp.update({str("Unique Id") : "RpAaSgSa"})
    pkg_shipp.update({str("Team Id") : "VB#1025"})
    

    var_handle_pub = rospy.Publisher('my_topic', myMessage, queue_size=10)
    var_loop_rate = rospy.Rate(1)
    
    
    pkg_shipp.update({str("Order ID") : list_of_list_2[0][1]})
    pkg_shipp.update({str("City") : list_of_list_2[0][2]})
    pkg_shipp.update({str("Item") : list_of_list_2[0][0]})
    pkg_shipp.update({str("Shipped Quantity") : "1"})
    pkg_shipp.update({str("Shipped Status") : "YES"})
    dte = self.get_time_str()
    pkg_shipp.update({str("Shipped Date and Time") : str(dte)})
    if list_of_list_2[0][0] == "Medicine" :
        pkg_shipp.update({str("Cost") : "300"})
        pkg_shipp.update({str("Priority") : "HP"})
        st = int(float(dte[8:10]))+ 1
        pkg_shipp.update({str("Estimated Time of Delivery") : str(dte[:8]) + str(st)})
        # x = dte[:10]
        # res = (datetime.strptime(x, '%Y-%m-%d') + timedelta(days=1)).strftime('%Y-%m-%d')
        # pkg_shipp.update({str("Estimated Time of Delivery") : str(res)})

    elif list_of_list_2[0][0] == "Food" :
        pkg_shipp.update({str("Cost") : "200"})
        pkg_shipp.update({str("Priority") : "MP"})
        st = int(float(dte[8:10]))+ 3
        # x = dte[:10]
        # res = (datetime.strptime(x, '%Y-%m-%d') + timedelta(days=3)).strftime('%Y-%m-%d')
        pkg_shipp.update({str("Estimated Time of Delivery") : str(dte[:8]) + str(st)})

    elif list_of_list_2[0][0] == "Clothes" :
        pkg_shipp.update({str("Cost") : "100"})
        pkg_shipp.update({str("Priority") : "LP"})
        st = int(float(dte[8:10])) +5
        # x = dte[:10]
        # res = (datetime.strptime(x, '%Y-%m-%d') + timedelta(days=3)).strftime('%Y-%m-%d')
        pkg_shipp.update({str("Estimated Time of Delivery") : str(dte[:8]) + str(st)})

    else :
        pass

    print(list_of_list_2)
    list_of_list_2.remove(list_of_list_2[0])
    print(list_of_list_2)
    obj_msg = myMessage()
    print("&&&&&&&&&&&&&&&&&&&&")
    print(pkg_shipp)
    #pkg_dict = message_converter.convert_dictionary_to_ros_message('std_msgs/String', pkg_det)
    #pkg_dict = json.dumps(pkg_det, separators=(',', ':') , sort_keys=True)
    pkg_dit = str(pkg_shipp)
    print("1111")
    print(pkg_dit)
    obj_msg.pkg_dit = pkg_dit
    rospy.sleep(1)
    rospy.loginfo("Publishing:in  OrdersShippedpped sheet ")
    #rospy.loginfo(obj_msg)

    var_handle_pub.publish(obj_msg)
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

def func_callback_topic_my_topic(LogicalCameraImage):
  """
  The func_callback_topic_my_topic is the callback function of the logical camera 2 in which output values are coming and we are sending those values to sort_pkg function.
  """
  if len(LogicalCameraImage.models) == 1:
    #rospy.loginfo("Data Received: ( %s , %s)",LogicalCameraImage.models[0].type,LogicalCameraImage.models[0].pose)
    if LogicalCameraImage.models[0].type != "ur5":
      if LogicalCameraImage.models[0].type != LastPackageName["default"] or LastPackageName["default"] =="default":
        
        
        ic.sort_pkg(LogicalCameraImage.models[0].type, LogicalCameraImage.models[0].pose)   

  elif len(LogicalCameraImage.models) == 2:
    rospy.loginfo("Data Received: ( %s , %s)",LogicalCameraImage.models[1].type,LogicalCameraImage.models[1].pose)
    if LogicalCameraImage.models[1].type != LastPackageName["default"] or LastPackageName["default"] =="default":
       
        
        ic.sort_pkg(LogicalCameraImage.models[1].type, LogicalCameraImage.models[1].pose)

  else :
    pass
    

def func_callback_topic_your_topic(obj_msg_2):
    """
    This function is the callback function of selfmade rostopic named as your_topic and in this function we are appending those values in the global list for further usage.
    """

    # rospy.loginfo("Data Received: (%d, %d, %d )", obj_msg.final_x,
    #               obj_msg.final_y, obj_msg.final_theta)
    # goal_handle['x']=obj_msg.final_x
    # goal_handle['y']=obj_msg.final_y
    # goal_handle['theta']=obj_msg.final_theta
    # action_server.on_goal(goal_handle)
    print("!!!!!!!!!!@@@@@@@@@@")
    print(type(obj_msg_2.pkg_dit))

    print(obj_msg_2.pkg_dit)
    data = eval(obj_msg_2.pkg_dit)
    print("list of req elements is "+ str(data))
    list_of_list_2.append(data)
    print(list_of_list_2)
    #goal_handle['pkg_dit']= data
    # #data = message_converter.convert_ros_message_to_dictionary(obj_msg)
    # msg = obj_msg.read()
    # data = json.loads(msg)
    # print("Type",type(data))
    
    # print("00000")
    #print(data)
    # print(goal_handle)
    # action_server.on_goal(goal_handle)
    ###### print("11111")

def main(args):
  rospy.sleep(80)
  rospy.init_node('node_eg1_read_camera', anonymous=True)
  global ic
 
  
  ic = Camera1("ur5_2")
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
	#rospy.sleep(110)
    main(sys.argv)
