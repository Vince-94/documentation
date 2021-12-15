#!/usr/bin/env python
# license removed for brevity

'''
    Node: tf2_broadcaster_node

    Broadcaster:	static_transform (static)
'''

import rospy
from tf2_ros import StaticTransformBroadcaster
from tf import transformations



def static_broadcaster():

    # Initilizing node
    rospy.init_node('static_broadcaster_node', anonymous=True)

    # Creating a transform object
    static_transform = geometry_msgs.msg.TransformStamped()

    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = ... # parent frame ("world", "odom", ...)
    static_transform.child_frame_id = ... # child frame

    static_transform.transform.translation.x = ... # position.x
    static_transform.transform.translation.y = ... # position.y
    static_transform.transform.translation.z = ... # position.z
    # Rotation: Euler RPY to quaternion
    q = tf.transformations.quaternion_from_euler(
        float(roll),
        float(pitch),
        float(yaw))
    static_transform.transform.rotation.x = q[0]
    static_transform.transform.rotation.y = q[1]
    static_transform.transform.rotation.z = q[2]
    static_transform.transform.rotation.w = q[3]

    # Creating a broadcaster object
    broadcaster = StaticTransformBroadcaster()
    broadcaster.sendTransform(static_transform)


    rospy.spin()


if __name__ == '__main__':
    try:
       static_broadcaster()
    except rospy.ROSInterruptException:
       pass
