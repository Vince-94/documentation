#!/usr/bin/env python
# license removed for brevity

'''
    Node: tf2_listener_node


'''

import rospy
from tf2_ros import Buffer, TransformListener


def listener():

	# Initializing the node
	rospy.init_node('tf2_turtle_listener')

	# Creating the listener object
	tfBuffer = Buffer()
    listener = TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform(
            											"target_frame",						# target frame
            											rospy.Time.now(),						# (option) target time
            											"source_frame",						# source frame
            											rospy.Time.now()-rospy.Duration(5.0),	# source time
            											"world",								# (option) fixed frame
            											rospy.Duration(1.0)						# (option) timeout
            											)

			rate.sleep()


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue


if __name__ == '__main__':
	listener()