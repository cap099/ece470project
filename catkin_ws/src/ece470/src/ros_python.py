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
from controller import Robot, Motor
import os
import time



def callback(data):
    global velocity
    global message
    message = 'Received velocity value: ' + str(data.data)
    velocity = data.data


robot = Robot()
# timeStep = int(robot.getBasicTimeStep())

timeStep = 32

left_back = robot.getDevice('back_left_wheel_joint')
left_front = robot.getDevice('front_left_wheel_joint')
right_front = robot.getDevice('front_right_wheel_joint')
right_back = robot.getDevice('back_right_wheel_joint')


left_back.setPosition(float('inf'))  # turn on velocity control for motors
left_front.setPosition(float('inf'))
right_front.setPosition(float('inf'))
right_back.setPosition(float('inf'))


left_back.setVelocity(0.0)  
left_front.setVelocity(0.0)
right_front.setVelocity(0.0)
right_back.setVelocity(0.0)



# sensor = robot.getDistanceSensor('prox.horizontal.2')  # front central proximity sensor
# sensor.enable(timeStep)

message = ''
print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
robot.step(timeStep)
rospy.init_node('listener', anonymous=True)
print('Subscribing to "motor" topic')
robot.step(timeStep)
rospy.Subscriber('motor', Float64, callback)
pub = rospy.Publisher('sensor', Float64, queue_size=10)



F = 2.0   # frequency 2 Hz
t = 0.0   # elapsed simulation time


time.sleep(3)
while robot.step(timeStep) != -1:
    velocity = 3.0
    left_back.setVelocity(velocity)  
    left_front.setVelocity(velocity)
    right_front.setVelocity(velocity)
    right_back.setVelocity(velocity)
    t += timeStep / 1000.0
    pub.publish(velocity)





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
