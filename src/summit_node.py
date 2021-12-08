import numpy as np
import rospy
from rospy.core import is_shutdown
from std_msgs.msg import Float64
from controller import Robot, Camera
import os
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


        self.imu = self.robot.getDevice('imu')
        self.imu.enable(self.timeStep)



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
        while self.robot.step(self.timeStep) != -1 and not rospy.is_shutdown() and self.front_distance_sensor.getValue() > 5:
            self.setVelocity(self.velo, 'left')

        while self.robot.step(self.timeStep) != -1 and not rospy.is_shutdown() and self.front_distance_sensor.getValue() > 0.25:
            self.setVelocity(self.velo, 'forward')


    def moveFromShelf(self):
        while self.robot.step(self.timeStep) != -1 and not rospy.is_shutdown() and self.left_distance_sensor.getValue() < 13.5:
            self.setVelocity(self.velo, 'right')


    def nextAisle(self):
        init_position = self.front_distance_sensor.getValue()
        while init_position - self.front_distance_sensor.getValue() < 2.5 and self.robot.step(self.timeStep) != -1:
            self.setVelocity(self.velo, 'forward')

    def turn180(self, direction):
        if direction == 'left':
            m = 1
        elif direction == 'right':
            m = -1

        roll, pitch, yaw = self.imu.getRollPitchYaw()
        while self.robot.step(self.timeStep) != -1:
            if self.imu.getRollPitchYaw()[2] - yaw >= np.pi:
                break
            else:
                self.left_front.setVelocity(m*self.velo)
                self.left_back.setVelocity(m*self.velo)  
                self.right_front.setVelocity(-m*self.velo)
                self.right_back.setVelocity(-m*self.velo)

                print(self.imu.getRollPitchYaw())

            





        



    def run(self):
        self.turn180('left')
        # if 'biscuits' in self.groceryList:
        #     self.moveToShelf()
        #     self.moveFromShelf()

        # self.nextAisle()
        # if 'honey' in self.groceryList:
        #     print('TURN')
        #     print('TURN AGAIN')


        # if 'jam' in self.groceryList:
        #     self.moveToShelf()
        #     self.moveFromShelf()

        # self.nextAisle()
        # if 'cereal' in self.groceryList:
        #     print('TURN')
        #     print('TURN AGAIN')

        # if 'soda' in self.groceryList:
        #     self.moveToShelf()
        #     self.moveFromShelf()






        
        # while self.robot.step(self.timeStep) != -1 and not rospy.is_shutdown():# and self.left_distance_sensor.getValue() > 5:
        #     velocity = 2
        #     self.left_front.setVelocity(-velocity)
        #     self.left_back.setVelocity(velocity)  
        #     self.right_front.setVelocity(velocity)
        #     self.right_back.setVelocity(-velocity)
        #     pub.publish(self.left_distance_sensor.getValue())

    # def callback(data):
    #     global velocity
    #     global message
    #     message = 'Received velocity value: ' + str(data.data)
    #     velocity = data.data





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
    pub = rospy.Publisher('sensor', Float64, queue_size=10)
    node = SummitNode(groceries)
    node.run()

    

