#! /usr/bin/env python

import rospy
import sys
import copy
import threading
import time 
import math

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import tf2_ros
import tf2_msgs.msg
from pkg_vb_sim.srv import conveyorBeltPowerMsg 
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.msg import LogicalCameraImage


class CartesianPath:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg5_waypoints', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        self._curr_state = self._robot.get_current_state()

        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')


    

    
    def go_to_pose(self, arg_pose):

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

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan


    def add_box(self,a,b,c, timeout=4):
        
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self._box_name
        scene = self._scene

        
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
        

    def attach_box(self, timeout=4):
       
       
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
        grasping_group = 'ur5_1_planning_group'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)
    
    def detach_box(self, timeout=4):
        
       
        box_name = self._box_name
        scene = self._scene
        eef_link = self._eef_link

        
        ##
        ## Detaching Objects from the arm
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        
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
    

    def activate_conveyer(self,val):
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        
        try:
            activate_conveyer_belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            
            resp1 = activate_conveyer_belt(val)
            print("Conveyer starts")
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)    

    def activate_gripper(self,flag):
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        
        try:
            activate_vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
            
            resp1 = activate_vacuum_gripper(flag)
            
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)  

    def deactivate_gripper(self,flag):
        
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        
        try:
            activate_vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
            
            resp1 = activate_vacuum_gripper(flag)
            
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)  


    
        
    


    def func_tf_print(self,arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())
            rospy.loginfo(  "\n" +
                            "Translation: \n" +
                            "x: {} \n".format(trans.transform.translation.x) +
                            "y: {} \n".format(trans.transform.translation.y) +
                            "z: {} \n".format(trans.transform.translation.z) +
                            "\n" +


                            "Orientation: \n" +
                            "x: {} \n".format(trans.transform.rotation.x) +
                            "y: {} \n".format(trans.transform.rotation.y) +
                            "z: {} \n".format(trans.transform.rotation.z) +
                            "w: {} \n".format(trans.transform.rotation.w) )
            # Condition to stop the conveyor when package arrives in the given range of position
            if float(trans.transform.translation.y) >= float(-0.109524240865) and float(trans.transform.translation.y) <= float(0):

                    
                    val2 = 0
                    ur5.activate_conveyer(val2)         
                    
                    # Home position declaration
                    ur5_2_home_pose = geometry_msgs.msg.Pose()
                    ur5_2_home_pose.position.x = -0.8
                    ur5_2_home_pose.position.y = 0
                    ur5_2_home_pose.position.z = 1 + 0.20 
                    ur5_2_home_pose.orientation.x = -0.5
                    ur5_2_home_pose.orientation.y = -0.5
                    ur5_2_home_pose.orientation.z = 0.5
                    ur5_2_home_pose.orientation.w = 0.5

                    # Condition to detect the RED package
                    if trans.transform.translation.x >= -0.840000 and trans.transform.translation.x <= -0.800000 :
                        # Position to pick the RED package
                        ur5_pose_2 = geometry_msgs.msg.Pose()                       
                        ur5_pose_2.position.x = trans.transform.translation.x
                        ur5_pose_2.position.y = trans.transform.translation.y
                        ur5_pose_2.position.z = trans.transform.translation.z + 0.20

                        ur5_pose_2.orientation.x = -0.5
                        ur5_pose_2.orientation.y = -0.5
                        ur5_pose_2.orientation.z = 0.5
                        ur5_pose_2.orientation.w = 0.5
                        ur5.go_to_pose(ur5_pose_2)
                        
                        # Position to add package in RVIZ 
                        a = trans.transform.translation.x
                        b = trans.transform.translation.y
                        c = trans.transform.translation.z
                        flag = True
                        # threads to ADD and ATTACH the box in RVIZ
                        t4 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
                        t5 = threading.Thread(target=ur5.attach_box, args=[])
                        # thread to activate the Vaccum Gripper in Gazebo
                        t6 = threading.Thread(target=ur5.activate_gripper, args=[flag])     

                        t4.start() 
                        t5.start()
                        t6.start()

                        rospy.loginfo('\033[94m' + "Translating EE to the package from current position for RED package." + '\033[0m')
                        # Joint angle values to reach the RED bin to place the RED package
                        lst_joint_angles_1 = [math.radians(50),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]
                        ur5.set_joint_angles(lst_joint_angles_1)
                        
                        flag = False
                        #threads to detach and remove the box in RVIZ 
                        t7 = threading.Thread(target=ur5.detach_box, args=[]) 
                        t8 = threading.Thread(target=ur5.remove_box, args=[])
                        # Thread to deactivate the Vaccum Gripper in Gazebo
                        t9 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

                        t7.start() 
                        t8.start()
                        t9.start()

                        # To start the conveyor to bring the Green package inside the logical camera frustum
                        val3 = 60
                        tcon2 = threading.Thread(target= ur5.activate_conveyer, args=[val3])
                        thome2 = threading.Thread(target= ur5.go_to_pose, args=[ur5_2_home_pose])
                    
                        tcon2.start()

                        thome2.start()

                    # Condition to detect the GREEN package
                    elif trans.transform.translation.x >= -0.700000 and trans.transform.translation.x <= -0.660000 :
                        # Home position declaration
                        ur5_pose_2 = geometry_msgs.msg.Pose()
                        ur5_pose_2.position.x = trans.transform.translation.x
                        ur5_pose_2.position.y = trans.transform.translation.y
                        ur5_pose_2.position.z = trans.transform.translation.z + 0.20
                        ur5_pose_2.orientation.x = -0.5
                        ur5_pose_2.orientation.y = -0.5
                        ur5_pose_2.orientation.z = 0.5
                        ur5_pose_2.orientation.w = 0.5
                        ur5.go_to_pose(ur5_pose_2)
                        
                        # Position to add package in RVIZ 
                        a = trans.transform.translation.x
                        b = trans.transform.translation.y
                        c = trans.transform.translation.z
                        flag = True
                        # threads to ADD and ATTACH the box in RVIZ
                        t4 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
                        t5 = threading.Thread(target=ur5.attach_box, args=[])
                        # thread to activate the Vaccum Gripper in Gazebo
                        t6 = threading.Thread(target=ur5.activate_gripper, args=[flag])

                        t4.start() 
                        t5.start()
                        t6.start()

                        # Joint angle values to reach the Green bin to place the Green package
                        lst_joint_angles_1 = [math.radians(-15),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]
                        rospy.loginfo('\033[94m' + "Translating EE to the package from current position for GREEN package." + '\033[0m')
                        ur5.set_joint_angles(lst_joint_angles_1)

                        flag = False
                        #threads to detach and remove the box in RVIZ
                        t7 = threading.Thread(target=ur5.detach_box, args=[]) 
                        t8 = threading.Thread(target=ur5.remove_box, args=[])
                        # Thread to deactivate the Vaccum Gripper in Gazebo
                        t9 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

                        t7.start() 
                        t8.start()
                        t9.start()

                        # To start the conveyor to bring the Blue package inside the logical camera frustum
                        val4 = 50
                        tcon2 = threading.Thread(target= ur5.activate_conveyer, args=[val4])
                        thome2 = threading.Thread(target= ur5.go_to_pose, args=[ur5_2_home_pose])
                    
                        tcon2.start()

                        thome2.start()

                    # Condition to detect the GREEN package
                    elif trans.transform.translation.x >= -0.940000 and trans.transform.translation.x <= -0.900000 :
                        # Home position declaration
                        ur5_pose_2 = geometry_msgs.msg.Pose()
                        ur5_pose_2.position.x = trans.transform.translation.x
                        ur5_pose_2.position.y = trans.transform.translation.y
                        ur5_pose_2.position.z = trans.transform.translation.z + 0.20
                        ur5_pose_2.orientation.x = -0.5
                        ur5_pose_2.orientation.y = -0.5
                        ur5_pose_2.orientation.z = 0.5
                        ur5_pose_2.orientation.w = 0.5
                        ur5.go_to_pose(ur5_pose_2)
                        # Position to add package in RVIZ 
                        a = trans.transform.translation.x
                        b = trans.transform.translation.y
                        c = trans.transform.translation.z
                        flag = True

                        # threads to ADD and ATTACH the box in RVIZ
                        t4 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
                        t5 = threading.Thread(target=ur5.attach_box, args=[])
                        # thread to activate the Vaccum Gripper in Gazebo
                        t6 = threading.Thread(target=ur5.activate_gripper, args=[flag])

                        t4.start() 
                        t5.start()
                        t6.start()

                        # Joint angle values to reach the Green bin to place the Green package
                        lst_joint_angles_1 = [math.radians(-100),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]
                        rospy.loginfo('\033[94m' + "Translating EE to the package from current position for BLUE package." + '\033[0m')
                        ur5.set_joint_angles(lst_joint_angles_1)
                        
                        flag = False
                        #threads to detach and remove the box in RVIZ
                        t7 = threading.Thread(target=ur5.detach_box, args=[]) 
                        t8 = threading.Thread(target=ur5.remove_box, args=[])
                        # thread to deactivate the Vaccum Gripper in Gazebo
                        t9 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

                        t7.start() 
                        t8.start()
                        t9.start()
           
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')

def func_callback_topic_my_topic(LogicalCameraImage):
        
        if len(LogicalCameraImage.models) != 0:
        
            rospy.loginfo("Data Received: ( %s , %s)",LogicalCameraImage.models[0].type,LogicalCameraImage.models[0].pose)
            # Threads to convert package position from logical camera frame to the world frame 

            t1 = threading.Thread(target=ur5.func_tf_print, args=("world", "logical_camera_2_packagen1_frame"))  #  For Red package 
            t2 = threading.Thread(target=ur5.func_tf_print, args=("world", "logical_camera_2_packagen2_frame"))  #  For Green package
            t3 = threading.Thread(target=ur5.func_tf_print, args=("world", "logical_camera_2_packagen3_frame"))  #  For Blue package
            t1.start() 
            t2.start()
            t3.start()

            t1.join() 
            t2.join()
            t3.join()
           
        
def main():
    global ur5
    ur5 = CartesianPath()

    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2)  # 0.19
    
    val1 = 60
    
    

    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0
    ur5_2_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
    # This to keep EE parallel to Ground Plane
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5

    tcon1 = threading.Thread(target= ur5.activate_conveyer, args=[val1])        # thread to activate the conveyor
    thome1 = threading.Thread(target= ur5.go_to_pose, args=[ur5_2_home_pose])   # thread for arm to go to home pose
    tcon1.start()
    thome1.start()

    # 2. Subscribe to the desired topic and attach a Callback Funtion to it.
    rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, func_callback_topic_my_topic)
    rospy.spin()    
    del ur5


if __name__ == '__main__':
    main()