#!/usr/bin/env python3

# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This is a simple example of a Webots controller running a Python ROS node thanks to rospy.
The robot is publishing the value of its front distance sensor and receving motor commands (velocity).
"""

import rospy
from std_msgs.msg import Float64
from controller import Robot
import os
import time
import numpy as np
from lab6_func import *


os.environ['WEBOTS_ROBOT_NAME'] = 'UR3e'
robot = Robot()
joint1 = robot.getDevice('shoulder_pan_joint')
joint2 = robot.getDevice('shoulder_lift_joint')
joint3 = robot.getDevice('elbow_joint')
joint4 = robot.getDevice('wrist_1_joint')
joint5 = robot.getDevice('wrist_2_joint')
joint6 = robot.getDevice('wrist_3_joint')
joints = [joint1, joint2, joint3, joint4, joint5, joint6]

hand_motors = [0]*3

hand_motors[0] = robot.getDevice("finger_1_joint_1")
hand_motors[1] = robot.getDevice("finger_2_joint_1")
hand_motors[2] = robot.getDevice("finger_middle_joint_1")


def move_arm(joints, thetas):
    for i,joint in enumerate(joints):
        if i == 0:
            joint.setPosition(thetas[i] + np.pi)
        elif i == 3:
            joint.setPosition(thetas[i] - np.pi/2)
        else:
            joint.setPosition(thetas[i])

def close_hand():
    global hand_motors
    for motor in hand_motors:
        motor.setPosition(0.85)


over_basket = [np.pi/5.5, -np.pi/2, -np.pi/8, -np.pi*3/8,np.pi/2,0]
test_thetas = [np.pi/2, 0, 0, -90, 0, 0]
zero_thetas = [0,0,0,0,0,0]
# thetas = lab_invk(0.25,0.15,-0.04,0)
thetas = lab_invk(0.25,0.15,0.25,0)



timeStep = 32
while robot.step(timeStep) != -1:
    print(thetas)
    move_arm(joints, over_basket)
    # close_hand()




# print('Running the control loop')
# while robot.step(timeStep) != -1 and not rospy.is_shutdown():
#     # pub.publish(sensor.getValue())
#     # print('Published sensor value: ', sensor.getValue())
#     if message:
#         print(message)
#         message = ''
#     left_back.setPosition(velocity)
#     left_front.setPosition(velocity)
#     right_front.setPosition(velocity)
#     right_back.setPosition(velocity)
