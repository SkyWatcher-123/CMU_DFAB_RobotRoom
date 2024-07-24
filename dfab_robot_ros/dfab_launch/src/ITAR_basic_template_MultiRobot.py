#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

"""
This is the Movit python interface template for communication between ROS and COMPASS-grasshopper plugin. It is specifcally for connecting both IRB6640 and IRB4400 in DFAB. Note that IRB4400 is available for I/O insturction

Before running this file, please make sure:
    1. ABB IRB6640 and IRB4400 Moveit is running (robot model is visible in rviz, no error in terminal, when using joystick to control the robot, rviz shows movement synchronization)
    2. ROS bridge is launched, so grasshopper are subscripted to the proper rostopic

The template offers the following basic functions:
    Robot Control Object
    Receive from Grasshopper: 
        1. End-Effector Tool0 goal pose (Point + Quaternion)
        3. Robot goal pose
        4. Mesh geometries (mesh geometries not be saved in local)
        5. Attach/Detach mesh geometries to IRB120 that are sent from grasshopper or loaded from rviz
        6. Optional arudino actuation for End-Effector
    Send to Grasshopper:
        1. Robot JointState (realtime synchronization)
        2. End-effector current state (Detached/Attached)
        3. waypoints(?)

Please be cautious when running this file and use ABB teaching pendant manual mode for safety test after code editing. 
"""


# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import serial
import time
import json
import os
from ITAR_template_utils import *
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32MultiArray, Bool
from shape_msgs.msg import Mesh
import numpy as np
from stl import mesh


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterfaceirb120"""

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("grasshopper_commander", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        group_name = "dfab_multirobot"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.allow_replanning(True)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        # Misc variables
        self.object_name = ""
        self.mesh_name = ""
        if not os.path.exists("mesh_list.json"):
            self.mesh_list={}
            with open('mesh_list.json', 'w') as outfile:  
                json.dump(self.mesh_list, outfile)
        else: 
            with open('mesh_list.json',"r") as in_file:
                self.mesh_list = json.load(in_file)
        self.mesh_list = {}

        root_path = os.path.abspath(os.sep)
        self.store_path =  os.path.join(root_path,'/home/vina/ws_moveit/geometries')
        # if os.path.exists(self.store_path) and os.path.isdir(self.store_path):
        #     files = os.listdir(self.store_path)
        #     print("Files in directory:", files)
        # else:
        #     print("The directory does not exist or is not a directory")



        self.publishers = []# a list of publisher object
        self.current_mesh_id = 0
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.current_plan = None
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.branch_pose = geometry_msgs.msg.PoseStamped()

        self.move_group.set_planning_time(5)
        self.move_group.set_goal_position_tolerance(1e-4)
        self.move_group.set_goal_orientation_tolerance(1e-4)

    def go_to_joint_state(self,goal_list):

        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = goal_list[0]
        joint_goal[1] = goal_list[1]
        joint_goal[2] = goal_list[2]
        joint_goal[3] = goal_list[3]
        joint_goal[4] = goal_list[4]
        joint_goal[5] = goal_list[5] 

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def get_pose(self):
        move_group = self.move_group
        wpose = move_group.get_current_pose().pose
        print(wpose)

    def go_to_pose_goal(self,pose_goal,preview_trajectory=False):
        move_group = self.move_group
        pose_goal = pose_goal
        move_group.set_pose_target(pose_goal)

        plan_success, plan, planning_time, error_code = move_group.plan(joints = pose_goal)
        if plan_success == True:
            print(self.publishers[1])
            self.publishers[1].publish(Bool(data=True))
            print(f"plan_success: {plan_success}")
            irb120.display_trajectory(plan)
            # if input(f'Do you want to execute the plan?[y/n]') == 'y':
            #     irb120.move_group.execute(plan, wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        # print(f' current_pose:{current_pose}')
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, waypoints, scale=1):
        """
        parameter: waypoints (geometry_msgs/PoseArray)
        """
        waypoints = waypoints.poses

        move_group = self.move_group

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        if input(f'Do you want to execute the plan?[y/n]') == 'y':
            irb120.move_group.execute(plan, wait=True)

        return plan, fraction


    def display_trajectory(self, plan):

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

    def send_tcp_waypoints(self,plan):
        # Access trajectory points
        trajectory = plan.joint_trajectory

        # Extract end-effector waypoints
        waypoints = []
        for point in trajectory.points:
            # For example, assuming the end-effector pose is represented by the last 6 elements of the joint values
            end_effector_pose = point.positions[-6:]
            waypoints.append(end_effector_pose)

        tcp_waypoints = PoseArray()
        tcp_waypoints.poses = waypoints

        self.publisher[0].publish(tcp_waypoints)

        return 


    def execute_plan(self, plan):

        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def wait_for_state_update(
        self, mesh_is_known=False, mesh_is_attached=False, timeout=10
    ):

        mesh_name = "mesh_"+str(self.current_mesh_id)
        scene = self.scene

        
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([mesh_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = mesh_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (mesh_is_attached == is_attached) and (mesh_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def save_mesh(self, input_mesh, id, timeout=4):
        vertices = []
        for i in range(len(input_mesh.vertices)):
            vertices.append([input_mesh.vertices[i].x,
            input_mesh.vertices[i].y,
            input_mesh.vertices[i].z])
        vertices = np.array(vertices)
        faces = []
        for i in range(len(input_mesh.triangles)):
            faces.append(input_mesh.triangles[i].vertex_indices)
        faces = np.array(faces)

        new_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
        for i, f in enumerate(faces):
            for j in range(3):
                new_mesh.vectors[i][j] = vertices[f[j],:]

        file_name = 'mesh_'+str(id)+'.stl'
        file_path = os.path.join(self.store_path, file_name)
        new_mesh.save(file_path)



    def add_mesh(self, timeout=4):
        mesh_name = "mesh_"+str(self.current_mesh_id)
        self.mesh_list[self.current_mesh_id]=mesh_name
        with open('mesh_list.json', 'w') as outfile:  
            json.dump(self.mesh_list, outfile)
        scene = self.scene

        mesh_pose = geometry_msgs.msg.PoseStamped()
        mesh_pose.header.frame_id = "base_link"
        mesh_pose.pose.orientation.w = 1.0
        self.mesh_pose = mesh_pose

        file_name = mesh_name +'.stl'
        file_path = os.path.join(self.store_path, file_name)

        scene.add_mesh(mesh_name, mesh_pose,filename = file_path)

        # self.mesh_name = mesh_name

        return self.wait_for_state_update(mesh_is_known=True, timeout=timeout)



    def attach_mesh(self, mesh_name, timeout=4):
        # mesh_name = 'at_branch'
        # mesh_name = self.mesh_list[id]
        # branch_pose = self.branch_pose
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        grasping_group = "manipulator"
        touch_links = robot.get_link_names(group=grasping_group)

        scene.attach_mesh(eef_link, mesh_name, touch_links=touch_links)
        self.move_group.set_start_state_to_current_state()

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            mesh_is_attached=True, mesh_is_known=False, timeout=timeout
        )

    def detach_mesh(self, timeout=4):
        # mesh_name = self.mesh_name[id]
        scene = self.scene
        eef_link = self.eef_link
        scene.remove_attached_object(eef_link)
        self.move_group.set_start_state_to_current_state()
        return self.wait_for_state_update(
            mesh_is_known=True, mesh_is_attached=False, timeout=timeout
        )
    
"""
callback functions for gh_ROS communication
"""


def callback_joint_goal(msg):
    """
    This call back function pass specified joint goal from Grasshopper, return True if successfully executed
    """
    success = irb120.go_to_joint_state(msg.data)
    return success


def callback_pose_goal(msg):
    """
    This call back function pass specified end-effector pose from Grasshopper, return True if successfully executed.
    """

    success = irb120.go_to_pose_goal(msg)
    return success

def callback_picknplace(msg):
    goal_list = msg.data
    irb120.go_to_pose_goal(goal_list)
    if len(goal_list)==9:
        if input('press y and enter to change gripper opening') == "y":
            gripper(goal_list[7])
            time.sleep(5)
        if goal_list[8] == 0:
            #if the motion is set to 0, attach the mesh onto the gripper in the planning scene
            if input('press y and enter to attch') == "y":
                irb120.attach_mesh(irb120.current_mesh_id)
        elif goal_list[8] == 1:
            #if the motion is set to 1, dettach the mesh on the gripper and remove it from planning scene
            if input('press y and enter to detach') == "y":
                irb120.detach_mesh()

                # irb120.scene.remove_world_object(irb120.mesh_list[irb120.current_mesh_id])
    input(f'press enter to continue the planning motion')
        
    
def callback_mesh(msg):
    """
    current_mesh_id initialized as 0, increment by 1 for each new mesh, the first mesh is saved as mesh_1.stl
    """
    irb120.current_mesh_id += 1
    irb120.save_mesh(msg, irb120.current_mesh_id)
    status = irb120.add_mesh()
    print(f'status: {status}')
    return status


def callback_eef(msg):

    """
    This call back function pass specified boolean value from Grasshopper to sync the robot end-effector for attaching or detaching the workobject  
    """
    action = msg.data.split(" ")

    if action[0] == 'True':
        irb120.attach_mesh(action[1])
    if action[0] == 'False':
        irb120.detach_mesh()
    return

def callback_cartesian(msg):

    """
    This call back function pass specified cartesian waypoints from Grasshopper, currently it does not support velocity definition 
    """

    success = irb120.plan_cartesian_path(msg)
    return success

def callback_execute_plan(msg):
    """
    This call back function pass specified boolean value from Grasshopper to execute the planned motion (if a current plan exists)
    """

    if irb120.current_plan and msg.data == 'True':
        success = irb120.execute_plan(irb120.current_plan)
    return success
        

irb120 = MoveGroupPythonInterface()

if __name__ == "__main__":
    #provide planned trajectory visualization
    irb_tcp = rospy.Publisher("/tcp_trajectory", PoseArray, queue_size=10)
    bool_plan_success = rospy.Publisher("/bool_plan_success", Bool, queue_size=10)
    irb120.publishers.append(irb_tcp)
    irb120.publishers.append(bool_plan_success)

    rospy.Subscriber('/rhino_mesh', Mesh,callback_mesh)
    rospy.Subscriber("/joint_goal_gh", Float32MultiArray, callback_joint_goal)
    rospy.Subscriber("/pose_goal_gh", Pose, callback_pose_goal)
    rospy.Subscriber("/attach_detach", String, callback_eef)
    rospy.Subscriber("/cartesian_waypoints", PoseArray,callback_cartesian)
    rospy.Subscriber("/picknplace", Float32MultiArray, callback_picknplace)
    rospy.Subscriber("/Execute_Current_Plan", String, callback_execute_plan)
    rospy.spin()

#
