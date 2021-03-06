import numpy as np
import rospy
from rospy.core import is_shutdown
from std_msgs.msg import Float64
from controller import Robot, Camera
import os
from lab6_func import *
import time



class SummitNode():
    def __init__(self, groceryList):
        self.groceryList = groceryList
        print(self.groceryList)

        self.aisle = 0
        self.velo = 2
        
        self.robot = Robot()
        self.timeStep = int(self.robot.getBasicTimeStep())

        self.rgb_camera = Camera('rgb_camera')
        self.rgb_camera.enable(self.timeStep)

        self.depth_camera = self.robot.getDevice('depth_camera')
        self.depth_camera.enable(self.timeStep)

        self.left_back = self.robot.getDevice('back_left_wheel_joint')
        self.left_front = self.robot.getDevice('front_left_wheel_joint')
        self.right_front = self.robot.getDevice('front_right_wheel_joint')
        self.right_back = self.robot.getDevice('back_right_wheel_joint')

        self.left_back.setPosition(float('inf'))  # turn on velocity control for motors
        self.left_front.setPosition(float('inf'))
        self.right_front.setPosition(float('inf'))
        self.right_back.setPosition(float('inf'))

        self.left_back.setVelocity(0.0)  # set initial velocity
        self.left_front.setVelocity(0.0)
        self.right_front.setVelocity(0.0)
        self.right_back.setVelocity(0.0)

        self.left_distance_sensor = self.robot.getDevice('left_distance_sensor')
        self.left_distance_sensor.enable(self.timeStep)
        self.right_distance_sensor = self.robot.getDevice('right_distance_sensor')
        self.right_distance_sensor.enable(self.timeStep)
        self.front_distance_sensor = self.robot.getDevice('front_distance_sensor')
        self.front_distance_sensor.enable(self.timeStep)
        self.rear_distance_sensor = self.robot.getDevice('rear_distance_sensor')
        self.rear_distance_sensor.enable(self.timeStep)

        self.object_sensor = self.robot.getDevice('object_sensor')
        self.object_sensor.enable(self.timeStep)


        self.imu = self.robot.getDevice('imu')
        self.imu.enable(self.timeStep)

        self.gripCamera = self.robot.getDevice('gripCamera')
        self.gripCamera.enable(self.timeStep)



    def setVelocity(self, v, direction):
        if direction == 'left':
            self.left_front.setVelocity(-v)
            self.left_back.setVelocity(v)  
            self.right_front.setVelocity(v)
            self.right_back.setVelocity(-v)
        elif direction == 'right':
            self.left_front.setVelocity(v)
            self.left_back.setVelocity(-v)  
            self.right_front.setVelocity(-v)
            self.right_back.setVelocity(v)
        elif direction == 'forward':
            self.left_front.setVelocity(v)
            self.left_back.setVelocity(v)  
            self.right_front.setVelocity(v)
            self.right_back.setVelocity(v)
        elif direction == 'backward':
            self.left_front.setVelocity(-v)
            self.left_back.setVelocity(-v)  
            self.right_front.setVelocity(-v)
            self.right_back.setVelocity(-v)
        else:
            print('input a direction')



    def moveToShelf(self):
        while self.robot.step(self.timeStep) != -1 and not rospy.is_shutdown() and self.front_distance_sensor.getValue() > 0.25:
            self.setVelocity(self.velo, 'forward')


    def moveFromShelf(self):
        while self.robot.step(self.timeStep) != -1 and not rospy.is_shutdown() and self.left_distance_sensor.getValue() < 13.5:
            self.setVelocity(self.velo, 'right')


    def nextAisle(self):
        init_position = self.front_distance_sensor.getValue()
        while init_position - self.front_distance_sensor.getValue() < 2.5 and self.robot.step(self.timeStep) != -1:
            self.setVelocity(self.velo, 'forward')

    def turn180(self):
        pass

    def moveToItem(self):
        while self.robot.step(self.timeStep) != -1:
            self.setVelocity(self.velo, 'left')
            print(self.object_sensor.getValue())
            if self.object_sensor.getValue() < 1:
                break

    def moveFromItem(self):
        while self.robot.step(self.timeStep) != -1:
            self.setVelocity(self.velo, 'left')
            if self.object_sensor.getValue() > 3:
                break

    def run(self):
        self.moveToShelf()
        for i in range(3):
            while self.robot.step(self.timeStep) != -1:
                self.setVelocity(self.velo, 'left')
                if self.object_sensor.getValue() < 0.3:
                    self.setVelocity(0, 'left')
                    start = time.time()
                    while time.time() - start < 5:
                        self.robot.step(self.timeStep)

        



def read_grocery_list(filename):
    groceries = {}
    file = open(filename, 'r')
    while True:
        line = file.readline()
        if not line:
            break
        elif line[0] == '#':
            continue
        [item, num] = line.split()
        groceries[item.lower()] = int(num)
    return groceries


if __name__ == '__main__':
    groceries = read_grocery_list('grocery_list.txt')
    os.environ['WEBOTS_ROBOT_NAME'] = 'Summit-XL Steel'
    rospy.init_node('summit', anonymous=True)
    pub = rospy.Publisher('ur3_command', Float64, queue_size=10)
    node = SummitNode(groceries)
    node.run()

    

