#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from pkg_vb_sim.srv import vacuumGripper

class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)

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
        
        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

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
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def add_box(self, timeout=4):
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
        box_pose.pose.position.x = 0.03		
        box_pose.pose.position.y = 0.46 	
        box_pose.pose.position.z = 1.88 
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
    	grasping_group = 'ur5_1_planning_group'
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
    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = 0.00
    ur5_pose_1.position.y = 0.26
    ur5_pose_1.position.z = 1.90
    ur5_pose_1.orientation.x = 0
    ur5_pose_1.orientation.y = 0
    ur5_pose_1.orientation.z = 0
    ur5_pose_1.orientation.w = 0

    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x = -0.60
    ur5_pose_2.position.y = -0.07
    ur5_pose_2.position.z = 1.20
    ur5_pose_2.orientation.x = 0
    ur5_pose_2.orientation.y = 0
    ur5_pose_2.orientation.z = 0
    ur5_pose_2.orientation.w = 0

    lst_joint_angles_1 = [math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]

       


    ur5.add_box()
    rospy.sleep(2)

    ur5.go_to_pose(ur5_pose_1)
    rospy.sleep(2)
    
    flag = True
    ur5.activate_gripper(flag)
    rospy.sleep(2)

    ur5.attach_box()
    rospy.sleep(2)
    
    ur5.go_to_pose(ur5_pose_2)
    rospy.sleep(2)

    flag = False
    ur5.deactivate_gripper(flag)
    rospy.sleep(2)

    ur5.detach_box()
    rospy.sleep(2)

    ur5.remove_box()
    rospy.sleep(2)

    
    ur5.set_joint_angles(lst_joint_angles_1)
    rospy.sleep(2)
    


    del ur5


if __name__ == '__main__':
    main()
