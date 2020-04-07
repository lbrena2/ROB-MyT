#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import Range


class Controller2:

    def __init__(self):
        """Initialization."""

        # initialize the node
        rospy.init_node(
            'thymio_controller'  # name of the node
        )

        self.name = rospy.get_param('~robot_name')

        # log robot name to console
        rospy.loginfo('Controlling %s' % self.name)

        # create velocity publisher
        self.velocity_publisher = rospy.Publisher(
            self.name + '/cmd_vel',  # name of the topic
            Twist,  # message type
            queue_size=10  # queue size
        )

        # Sensors subscribers
        self.right_sensor_sub = rospy.Subscriber(
            self.name + '/proximity/right',
            Range,
            self.log_sensor,
            ('right')
        )

        self.left_sensor_sub = rospy.Subscriber(
            self.name + '/proximity/left',
            Range,
            self.log_sensor,
            ('left')
        )

        self.center_right_sensor_sub = rospy.Subscriber(
            self.name + '/proximity/center_right',
            Range,
            self.log_sensor,
            ('center_right')
        )

        self.center_left_sensor_sub = rospy.Subscriber(
            self.name + '/proximity/center_left',
            Range,
            self.log_sensor,
            ('center_left')
        )

        
        self.sensors_values = {
            'left': 0,
            'right' : 0,
            'center_right' : 0,
            'center_left': 0 
        }


        #max distance in meters 
        #max = 0.119999997318
        #min = 0.00999999977648
        self.max_sensor_distance = 0.119999997318
        self.min_sensor_distance = 0.00999999977648
        # set node update frequency in Hz        
        self.rate = rospy.Rate(10) 

        #/gazebo/model_states   

    def log_sensor(self, data, args):
        self.sensors_values[args] = round(data.range, 12)
        #rospy.loginfo("Sensor value %f", self.sensors_values[args])

    def get_control(self):
        return Twist(
            linear=Vector3(
                .1,  # moves forward .2 m/s
                .0,
                .0,
            ),
            angular=Vector3(
                .0,
                .0,
                .0
            )
        )

    def run(self):
        """Controls the Thymio.""" 
        velocity = self.get_control()
        stop = True   
        epsilon = 0.05
        steer_direction = ''
        
        while not rospy.is_shutdown() and stop:

            self.velocity_publisher.publish(velocity)
            self.rate.sleep()
            print(self.sensors_values)
            # All returns true iff all the values of the sensors are 0.12 (no obstacle)
            # Stops the robot when hit the wall
            if not all(value >= self.max_sensor_distance - epsilon for value in self.sensors_values.values()):
                print('mi fermo')
                self.stop()
                stop = False
                # Take the min_value to choose in which direction MyT must steer
                steer_direction = min(self.sensors_values, key = self.sensors_values.get) 
                print(steer_direction)                            
            
        #set the angular_vel in order to spin clockwise or anti
        velocity.linear.x = 0
        if steer_direction == 'right' or steer_direction == 'center_right':
            velocity.angular.z = -0.1
        else:
            velocity.angular.z = 0.1

        while abs(self.sensors_values['center_right'] - self.sensors_values['center_left']) > 0.001:
            self.velocity_publisher.publish(velocity)
            self.rate.sleep()

        self.stop()

    def stop(self):
        self.velocity_publisher.publish(
            Twist()  
        )
        self.rate.sleep()


if __name__ == '__main__':
    controller = Controller2()

    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
