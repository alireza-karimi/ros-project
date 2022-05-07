#!/usr/bin/python3

import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from math import radians, sqrt
import numpy as np


class PID:

    def __init__(self) -> None:
        
        rospy.init_node("pid" , anonymous=False)
        
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        
        # getting specified parameters
        self.k_i = rospy.get_param("/pid/k_i")
        self.k_p = rospy.get_param("/pid/k_p")
        self.k_d = radians(rospy.get_param("/pid/k_d"))
        
        self.dt = rospy.get_param("/pid/dt")
        # self.v = rospy.get_param("/pid/v")
        self.D = rospy.get_param("/pid/D")
        rate = 1/self.dt
        self.r = rospy.Rate(rate)

        ### defining the rectangle
        X1 = np.linspace(-3, 3 , 100)
        Y1 = np.array([2]*100)

        Y2 = np.linspace(2, -2 , 100)
        X2 = np.array([3]*100)

        X3 = np.linspace(3, -3 , 100)
        Y3 = np.array([-2]*100)

        Y4 = np.linspace(-2, 2 , 100)
        X4 = np.array([-3]*100)

        X = np.concatenate([X1,X2, X3 , X4]).tolist()
        Y = np.concatenate([Y1,Y2,Y3,Y4]).tolist()
        reactangle_tuple = (X, Y)

        self.rectangle = reactangle_tuple
        ###


        ### Error Saving
        self.errors = []
        self.counter = 0
        ###

    # checks whether thr robot is on the line or not
    def odom_callback(self, msg):

        position = msg.pose.pose.position
        nearest_x = self.nearest_dot(position.x, position.y)[0]
        nearest_y = self.nearest_dot(position.x, position.y)[1]
        distance = self.calculate_distance(position.x, position.y, nearest_x, nearest_y)
        self.errors.append(distance)

        self.counter += 1
        rospy.loginfo(self.counter)
            

    # finding the robot nearest path dot
    def nearest_dot(self, robot_x, robot_y):
        min_distance = 100000000000000
        nearest_x = 0
        nearest_y = 0

        for x, y in zip(*self.rectangle):
            distance = self.calculate_distance(x, y, robot_x, robot_y)
            if distance < min_distance:
                min_distance = distance
                nearest_x = x
                nearest_y = y

        return nearest_x, nearest_y, min_distance

    # calculating distance of two points
    def calculate_distance(self, x1, y1, x2, y2):
        return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) )

    # updating visualiztion
    def update_errors(self):

        with open("/home/alirezakarimi/Desktop/catkin_ws/src/ros_tutorial/src/error-data.txt", "w") as file:
            file_lines = "\n".join([str(round(i, 2)) for i in self.errors])
            file.write(file_lines)
            # rospy.loginfo("file saved!")

    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to euler
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw
    
    def get_position(self):

        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        position = msg.pose.pose.position

        return position

    def run(self):  

        sum_i_theta = 0
        prev_theta_error = 0

        twist = Twist()
        twist.angular.z = 0
        twist.linear.x = 0
        self.cmd_publisher.publish(twist)

        while not rospy.is_shutdown():

            self.update_errors()
            
            error = self.nearest_dot(self.get_position().x, self.get_position().y)[2] - self.D 
            rospy.loginfo(error)
            sum_i_theta += error * self.dt 

            P = self.k_p * error
            I = self.k_i * sum_i_theta
            D = self.k_d * (error - prev_theta_error)

            twist = Twist()
            twist.angular.z = P + I + D
            twist.linear.x = P + I + D
            self.cmd_publisher.publish(twist)

            prev_theta_error = error

            self.r.sleep()


if __name__ == "__main__":

    pid = PID()

    pid.run()
         