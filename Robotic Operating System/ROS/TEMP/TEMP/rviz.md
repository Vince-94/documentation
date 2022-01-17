**DO NOT open or modify CMakeLists.txt and package.xml files if you are not the developer!**

# Rviz

## markers

Topic messages
- visualization_msgs/Marker

``` python
    marker = visualization_msgs.Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time()
    marker.ns = "my_namespace"
    marker.id = 0
    marker.type = visualization_msgs.Marker.SHAPE # (0=SRROW, 1=CUBE, 2=SPHERE, 3=CYLINDER, 4=LINE_STRIP, 5=LINE_LIST, 6=CUBE_LIST, 7=SPHERE_LIST, 8=POINTS, 9=TEXT_VIEW_FACING, 10=MESH_RESOURCE, 11=TRIANGLE_LIST)
    marker.action = visualization_msgs.Marker.ADD
    marker.pose.position.x = 1
    marker.pose.position.y = 1
    marker.pose.position.z = 1
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0  # Don't forget to set to 1!
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    # only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae"
    vis_pub.publish(marker)
```

- visualization_msgs/MarkerArray: for publish many markers at one


