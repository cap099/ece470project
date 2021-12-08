import rospy
from std_msgs.msg import Float64
from controller import Robot, Motor, Camera
import os
import time

os.environ['WEBOTS_ROBOT_NAME'] = 'Summit-XL Steel'

def callback(data):
    global velocity
    global message
    message = 'Received velocity value: ' + str(data.data)
    velocity = data.data


robot = Robot()
timeStep = int(robot.getBasicTimeStep())

rgb_camera = Camera('rgb_camera')
rgb_camera.enable(33)

depth_camera = robot.getDevice('depth_camera')
depth_camera.enable(33)

left_back = robot.getDevice('back_left_wheel_joint')
left_front = robot.getDevice('front_left_wheel_joint')
right_front = robot.getDevice('front_right_wheel_joint')
right_back = robot.getDevice('back_right_wheel_joint')

left_back.setPosition(float('inf'))  # turn on velocity control for motors
left_front.setPosition(float('inf'))
right_front.setPosition(float('inf'))
right_back.setPosition(float('inf'))

left_back.setVelocity(0.0)  # set initial velocity
left_front.setVelocity(0.0)
right_front.setVelocity(0.0)
right_back.setVelocity(0.0)

left_distance_sensor = robot.getDevice('left_distance_sensor')
left_distance_sensor.enable(timeStep)

right_distance_sensor = robot.getDevice('right_distance_sensor')
right_distance_sensor.enable(timeStep)

pub = rospy.Publisher('sensor', Float64, queue_size=10)
rospy.init_node('listener', anonymous=True)


while robot.step(timeStep) != -1 and not rospy.is_shutdown() and left_distance_sensor.getValue() > 5:
    velocity = 2
    left_front.setVelocity(velocity)
    left_back.setVelocity(velocity)  
    right_front.setVelocity(velocity)
    right_back.setVelocity(velocity)
    pub.publish(left_distance_sensor.getValue())


