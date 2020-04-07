#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Twist, Vector3



class Controller1:

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

        # set node update frequency in Hz
        
        self.rate = rospy.Rate(10)   

    def get_control(self):
        return Twist(
            linear=Vector3(
                .2,  # moves forward .2 m/s
                .0,
                .0,
            ),
            angular=Vector3(
                .0,
                .0,
                np.pi/3
            )
        )

    def run(self):
        """Controls the Thymio."""
        velocity = self.get_control()
        revolution_time = abs(((np.pi * 2 ) / velocity.angular.z)) 
        while True:
            start = rospy.Time.now()
            while abs(start.secs -  rospy.Time.now().secs) < revolution_time  and not rospy.is_shutdown():
                rospy.loginfo('Start_time, now %f, %f', start.secs, rospy.Time.now().secs)
                self.velocity_publisher.publish(velocity)
                self.rate.sleep()
            velocity.angular.z = velocity.angular.z * (-1)


if __name__ == '__main__':
    controller = Controller1()

    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
