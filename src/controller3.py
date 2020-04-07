#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion



class Controller3:

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

        #TODO: Sistema allo stesso modo il controller 2
        #TODO: implementa la versione usando /odom
        self.sensors_list = ['right', 
        'left', 
        'center', 
        'center_right',
        'center_left',
        'rear_left',
        'rear_right'
        ]
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


        self.rear_left_sensor_sub = rospy.Subscriber(
            self.name + '/proximity/rear_left',
            Range,
            self.log_sensor,
            ('rear_left')
        )

        self.rear_right_sensor_sub = rospy.Subscriber(
            self.name + '/proximity/rear_right',
            Range,
            self.log_sensor,
            ('rear_right')
        )

         # create pose subscriber
        self.pose_subscriber = rospy.Subscriber(
            self.name + '/odom',  # name of the topic
            Odometry,  # message type
            self.log_odometry  # function that hanldes incoming messages
        )

        
        self.sensors_values = {
            'left': 0,
            'right' : 0,
            'center_right' : 0,
            'center_left': 0,
            'rear_right' : 0,
            'rear_left' : 0
        }

        # initialize pose to (X=0, Y=0, theta=0)
        self.pose = Pose()
        self.yaw = 0
        #max distance in meters 
        
        self.max_sensor_distance = 0.119999997318
        self.min_sensor_distance = 0.00999999977648
        # set node update frequency in Hz        
        self.rate = rospy.Rate(10) 

        #/gazebo/model_states 

    def human_readable_pose2d(self, pose):
        """Converts pose message to a human readable pose tuple."""
        #TO-KNOW: che vuol dire convertire dalla quaternion a euler rotation?
        # create a quaternion from the pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        # convert quaternion rotation to euler rotation
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.yaw = yaw

        result = (
            pose.position.x,  # x position
            pose.position.y,  # y position
            yaw  # theta angle
        )

        return result

    def log_odometry(self, data):
        """Updates robot pose and velocities, and logs pose to console."""

        self.pose = data.pose.pose
        self.velocity = data.twist.twist

        printable_pose = self.human_readable_pose2d(self.pose)

        # log robot's pose
        rospy.loginfo_throttle(
            period=1,  # log every 10 seconds
            msg=self.name + ' (%.3f, %.3f, %.3f) ' % printable_pose  # message
        )  

    def log_sensor(self, data, args):
        self.sensors_values[args] = round(data.range, 12)
        #rospy.loginfo("Sensor value %f", self.sensors_values[args])

    def get_control(self):
        return Twist(
            linear=Vector3(
                .1,  # moves forward .1 m/s
                .0,
                .0,
            ),
            angular=Vector3(
                .0,
                .0,
                .0
            )
        )

    def stop(self):
        self.velocity_publisher.publish(
            Twist()  
        )
        self.rate.sleep()

    def get_close(self):
        """Controls the Thymio.""" 
        velocity = self.get_control()
        stop = True   
        epsilon = 0.05
        steer_direction = ''
        
        while not rospy.is_shutdown() and stop:

            self.velocity_publisher.publish(velocity)
            self.rate.sleep()
            # All returns true iff all the values of the sensors are 0.12 (no obstacle)
            # Stops the robot when hit the wall
            if not all(value >= self.max_sensor_distance - epsilon for value in self.sensors_values.values()):
                print('mi fermo')
                self.stop()
                stop = False
                print(self.sensors_values)
                # Take the min_value to choose in which direction MyT must steer
                steer_direction = max(self.sensors_values, key = self.sensors_values.get) 
                print(steer_direction)                            
            
        #set the angular_vel in order to spin clockwise or anti
        velocity.linear.x = 0

        if (steer_direction == 'right' or 
            steer_direction == 'center_right' or 
            steer_direction == 'rear_right'):
            
            velocity.angular.z = -0.1
        
        else:

            velocity.angular.z = 0.1

        while (self.sensors_values['rear_right'] >= self.max_sensor_distance or 
                self.sensors_values['rear_left'] >= self.max_sensor_distance):
            # print('rear_right: ' + str(self.sensors_values['rear_right']))
            # print('rear_left: ' + str(self.sensors_values['rear_left']))
            self.velocity_publisher.publish(velocity)
            self.rate.sleep()

        while abs(self.sensors_values['rear_right'] - self.sensors_values['rear_left']) >= 0.001:
            self.velocity_publisher.publish(velocity)
            self.rate.sleep()

        print('sono perpendicolare')
        self.rate.sleep()
        self.stop()

        self.go_ahead_2m()


    def go_ahead_2m(self):
        target_distance = 2 - self.sensors_values['rear_left']
        vel_msg = self.get_control()
        final_time = target_distance / vel_msg.linear.x
        starting_time = rospy.Time.now().secs

        while abs(rospy.Time.now().secs - starting_time) <= final_time:
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()  
            
        self.stop() 


if __name__ == '__main__':
    controller = Controller3()

    try:
        controller.get_close()
    except rospy.ROSInterruptException as e:
        pass
