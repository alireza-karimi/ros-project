#!/usr/bin/python3

import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from math import radians, sqrt
import numpy as np


class Follower:

    def __init__(self) -> None:
        
        rospy.init_node("follower" , anonymous=False)
        
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        
        # getting specified parameters
        self.linear_speed = rospy.get_param("/follower/linear_speed") # m/s
        self.angular_speed = rospy.get_param("/follower/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/follower/goal_angle")) # rad
        self.stop_distance = rospy.get_param("/follower/stop_distance") # m
        self.epsilon = rospy.get_param("/follower/epsilon")
        
        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 

        # control variables for rotation
        self.north_west = 0
        self.south_west = 0
        self.south_east = 0
        self.north_east = 0

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
        nearest_x, nearest_y = self.nearest_dot(position.x, position.y)
        distance = self.calculate_distance(position.x, position.y, nearest_x, nearest_y)
        self.errors.append(distance)

        self.counter += 1
        rospy.loginfo(self.counter)

        if ( (self.north_west == 0) and ( ( (msg.pose.pose.position.x < -3) and (msg.pose.pose.position.y > 0) ) or ( (abs(msg.pose.pose.position.x - (-3)) <= self.stop_distance) and (abs(msg.pose.pose.position.y - (2)) <= self.stop_distance) ) ) ):
            rospy.loginfo("north_west")
            self.north_west = 1
            self.south_west = 0
            self.south_east = 0
            self.north_east = 0
            self.state = self.ROTATE
        elif( (self.south_west == 0) and ( ( (msg.pose.pose.position.x < 0) and (msg.pose.pose.position.y < -2) ) or ( (abs(msg.pose.pose.position.x - (-3)) <= self.stop_distance) and (abs(msg.pose.pose.position.y - (-2)) <= self.stop_distance) ) ) ):
            rospy.loginfo("south_west")
            self.north_west = 0
            self.south_west = 1
            self.south_east = 0
            self.north_east = 0
            self.state = self.ROTATE
        elif( (self.south_east == 0) and ( ( (msg.pose.pose.position.x > 3) and (msg.pose.pose.position.y < 0) ) or ( (abs(msg.pose.pose.position.x - (3)) <= self.stop_distance) and ( abs(msg.pose.pose.position.y - (-2)) <= self.stop_distance) ) ) ):
            rospy.loginfo("south_east")
            self.north_west = 0
            self.south_west = 0
            self.south_east = 1
            self.north_east = 0
            self.state = self.ROTATE
        elif( (self.north_east == 0) and ( ( (msg.pose.pose.position.x > 0) and (msg.pose.pose.position.y > 2) ) or ( (abs(msg.pose.pose.position.x - (3)) <= self.stop_distance) and (abs(msg.pose.pose.position.y - (2)) <= self.stop_distance) ) ) ): 
            rospy.loginfo("north_east")
            self.north_west = 0
            self.south_west = 0
            self.south_east = 0
            self.north_east = 1
            self.state = self.ROTATE
        elif( (self.north_west == 0) and (self.south_west == 0) and (self.south_east == 0) and (self.north_east == 0)):
            self.state = self.GO 
            

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

        return nearest_x, nearest_y

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

        while not rospy.is_shutdown():

            self.update_errors()

            # check whether state is changed or not
            if self.state == self.GO:
                twist = Twist()
                twist.linear.x = self.linear_speed
                self.cmd_publisher.publish(twist)
                continue
            
            self.cmd_publisher.publish(Twist())
            
            rospy.sleep(1)
            
            remaining = self.goal_angle
            prev_angle = self.get_heading()
            
            twist = Twist()
            twist.angular.z = self.angular_speed
            self.cmd_publisher.publish(twist)
            
            # rotation loop
            while remaining >= self.epsilon:
                current_angle = self.get_heading()
            
                # if direction is being changed we must ignore the iteration to avoid large delta
                if current_angle*prev_angle < 0:
                    prev_angle = current_angle
                    continue
                
                delta = abs(prev_angle - current_angle)
                remaining -= delta
                # rospy.loginfo( ("remaining, epsilon, delta, current_angle, prev_angle", remaining, self.epsilon, delta, current_angle, prev_angle) )
                prev_angle = current_angle

            self.cmd_publisher.publish(Twist())

            rospy.sleep(1)

            self.state = self.GO


if __name__ == "__main__":

    follower = Follower()

    ### going from center to nearest rectangle point
    # rotate()
    follower.cmd_publisher.publish(Twist())
            
    rospy.sleep(1)

    remaining = follower.goal_angle
    prev_angle = follower.get_heading()
    
    twist = Twist()
    twist.angular.z = follower.angular_speed
    follower.cmd_publisher.publish(twist)
    
    while remaining >= follower.epsilon:
        # rospy.loginfo( ("remaining is ...", remaining) )
        current_angle = follower.get_heading()
        delta = abs(prev_angle - current_angle)
        remaining -= delta
        prev_angle = current_angle
    
    follower.cmd_publisher.publish(Twist())

    rospy.sleep(1)

    # go()
    twist = Twist()
    twist.linear.x = follower.linear_speed
    follower.cmd_publisher.publish(twist)

    while( not ( (follower.get_position().y > 2) or ( (abs(follower.get_position().x - 0) <= follower.stop_distance) and (abs(follower.get_position().y - 2) <= follower.stop_distance) ) ) ):
        continue  

    follower.cmd_publisher.publish(Twist())
    rospy.sleep(1)


    # rotate()
    remaining = follower.goal_angle
    prev_angle = follower.get_heading()
    
    twist = Twist()
    twist.angular.z = follower.angular_speed
    follower.cmd_publisher.publish(twist)
    
    while remaining >= follower.epsilon:
        current_angle = follower.get_heading()
        delta = abs(prev_angle - current_angle)
        remaining -= delta
        prev_angle = current_angle
    
    follower.cmd_publisher.publish(Twist())

    rospy.sleep(1)

    # go()
    follower.state = follower.GO

    ###

    follower.run()
         