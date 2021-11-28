#!/usr/bin/env python
# license removed for brevity

'''
    Node: tf2_broadcaster_node

    Subscriber to:
        /topic (pkg_msgs.msg.module)

    Broadcaster:	dynamic_transform
'''

import rospy
from tf2_ros import TransformBroadcaster
from tf import transformations
from geometry_msgs.msg import TransformStamped
from pkg_msgs.msg import module



def callback(data_msg):
   	# Creating a broadcaster object
    broadcaster = TransformBroadcaster()

	# Creating a message object
    dynamic_transform = TransformStamped()
    # Assigning variables
    dynamic_transform.header.stamp = rospy.Time.now()
    dynamic_transform.header.frame_id = ... # parent frame ("world", "odom", ...)
    dynamic_transform.child_frame_id = ... # child frame
    dynamic_transform.transform.translation.x = data_msg... # position.x
    dynamic_transform.transform.translation.y = data_msg... # position.y
    dynamic_transform.transform.translation.z = data_msg... # position.z

    # Rotation
    q = tf.transformations.quaternion_from_euler(
        float(roll),
        float(pitch),
        float(yaw))
    dynamic_transform.transform.rotation.x = q[0]
    dynamic_transform.transform.rotation.y = q[1]
    dynamic_transform.transform.rotation.z = q[2]
    dynamic_transform.transform.rotation.w = q[3]

    # Sending transform
    broadcaster.sendTransform(dynamic_transform)


def broadcaster():

	# Initializing the node
    rospy.init_node('tf2_broadcaster_node', anonymous=True)

    # Subscribe to "turtleX/pose" and run function handle_turtle_pose on every incoming message
    rospy.Subscriber('/topic', module, callback)

    # Keeping python from exiting until the node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
       broadcaster()
    except rospy.ROSInterruptException:
       pass




