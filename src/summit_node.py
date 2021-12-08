import rospy
from std_msgs.msg import Float64
from controller import Robot, Camera
import os


class SummitNode():
    def __init__(self):
        self.robot = Robot()
        self.timeStep = int(self.robot.getBasicTimeStep())

        self.rgb_camera = Camera('rgb_camera')
        self.rgb_camera.enable(33)

        self.depth_camera = self.robot.getDevice('depth_camera')
        self.depth_camera.enable(33)

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

    def run(self):
        while self.robot.step(self.timeStep) != -1 and not rospy.is_shutdown() and self.left_distance_sensor.getValue() > 5:
            velocity = 2
            self.left_front.setVelocity(velocity)
            self.left_back.setVelocity(velocity)  
            self.right_front.setVelocity(velocity)
            self.right_back.setVelocity(velocity)
            pub.publish(self.left_distance_sensor.getValue())

    def callback(data):
        global velocity
        global message
        message = 'Received velocity value: ' + str(data.data)
        velocity = data.data



if __name__ == '__main__':
    os.environ['WEBOTS_ROBOT_NAME'] = 'Summit-XL Steel'
    rospy.init_node('summit', anonymous=True)
    pub = rospy.Publisher('sensor', Float64, queue_size=10)
    node = SummitNode()
    node.run()

