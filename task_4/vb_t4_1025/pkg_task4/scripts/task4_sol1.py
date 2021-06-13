#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

import threading
import yaml
import os
import math
import time
import sys
import copy

from pkg_vb_sim.srv import conveyorBeltPowerMsg 
from std_srvs.srv import Empty
from pkg_vb_sim.srv import vacuumGripper


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
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories_shelf/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    

    def set_joint_angles(self, arg_list_joint_angles):

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

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()

    

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()
        
        return True


    def add_box(self,a,b,c, timeout=4):
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
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        
        try:
            activate_vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
            
            resp1 = activate_vacuum_gripper(flag)
            
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)  

    def deactivate_gripper(self,flag):
        
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        
        try:
            activate_vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
            
            resp1 = activate_vacuum_gripper(flag)
            
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)  

    def activate_conveyer(self,val):
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        
        try:
            activate_conveyer_belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            
            resp1 = activate_conveyer_belt(val)
            print("Conveyer starts")
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)    

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    rospy.sleep(95)
    ur5 = Ur5Moveit("ur5_1")

    lst_joint_angles_home_place = [math.radians(172),
                          math.radians(-46),
                          math.radians(59),
                          math.radians(-109),
                          math.radians(-90),
                          math.radians(0)]

    # saved trajectory play for package00
    a = 0.28
    b = -0.41
    c = 1.91
    flag = True
    rospy.logwarn("1. Playing allzeros to package00 (red) Trajectory File")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '1_allzeros_to_pkg00_red1.yaml', 5)
    
    t1 = threading.Thread(target=ur5.add_box, args=[a,b,c]) 
    t2 = threading.Thread(target=ur5.attach_box, args=[])
    t3 = threading.Thread(target=ur5.activate_gripper, args=[flag])
    t1.start() 
    t2.start()
    t3.start()
    

    flag = False
    rospy.logwarn("2. Playing package00 to the home pose  Trajectory File to place package")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '1_Pkg00_red_place1.yaml', 5)
    
    t4 = threading.Thread(target=ur5.detach_box, args=[]) 
    t5 = threading.Thread(target=ur5.remove_box, args=[])
    # thread to deactivate the Vaccum Gripper in Gazebo
    t6 = threading.Thread(target=ur5.deactivate_gripper, args=[flag])

    t4.start() 
    t5.start()
    t6.start()

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


    


    # saved trajectory play for package21
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


    # saved trajectory play for package22
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
        

    del ur5



if __name__ == '__main__':
    main()
