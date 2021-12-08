import rospy
from std_msgs.msg import Float64
from controller import Robot, Camera
import os
import numpy as np
from lab6_func import *



down = lab_invk(0.25,0.15,-0.03,0)
up = lab_invk(0.25,0.15,0.25,0)

class UR3Node():
    def __init__(self):
        self.robot = Robot()
        self.timeStep = int(self.robot.getBasicTimeStep())
        self.joint1 = self.robot.getDevice('shoulder_pan_joint')
        self.joint2 = self.robot.getDevice('shoulder_lift_joint')
        self.joint3 = self.robot.getDevice('elbow_joint')
        self.joint4 = self.robot.getDevice('wrist_1_joint')
        self.joint5 = self.robot.getDevice('wrist_2_joint')
        self.joint6 = self.robot.getDevice('wrist_3_joint')
        self.joints = [self.joint1, self.joint2, self.joint3, self.joint4, self.joint5, self.joint6]

        self.hand_motors = [0]*3
        self.hand_motors[0] = self.robot.getDevice("finger_1_joint_1")
        self.hand_motors[1] = self.robot.getDevice("finger_2_joint_1")
        self.hand_motors[2] = self.robot.getDevice("finger_middle_joint_1")

        self.over_basket = [np.pi/10, -3*np.pi/4, -np.pi/8, np.pi/8,np.pi/2,0]
        self.straight_up  = [0,-np.pi/2,0,0,0,0]
        self.zero_thetas = [0,0,0,0,0,0]
        # self.view_shelf = [0,-np.pi/2,np.pi/3,np.pi/6,-np.pi/2,np.pi/2]
        self.view_shelf = lab_invk(0.15,0.15, 0.2, 0)

        self.gripCamera = self.robot.getDevice('gripCamera')
        self.gripCamera.enable(self.timeStep)




    def run(self):
        while self.robot.step(self.timeStep) != -1 and not rospy.is_shutdown():
            self.move_arm(self.view_shelf)


    def move_arm(self, thetas):
        for i,joint in enumerate(self.joints):
            if i == 0:
                joint.setPosition(thetas[i] + np.pi)
            elif i == 3:
                joint.setPosition(thetas[i] - np.pi/2)
            else:
                joint.setPosition(thetas[i])

    def close_hand(self):
        for motor in self.hand_motors:
            motor.setPosition(0.85)

    def open_hand(self):
        for motor in self.hand_motors:
            motor.setPosition(0.0)




if __name__ == '__main__':
    os.environ['WEBOTS_ROBOT_NAME'] = 'UR3e'
    rospy.init_node('ur3', anonymous=True)
    node = UR3Node()
    node.run()
