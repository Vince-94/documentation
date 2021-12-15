#!/usr/bin/env python

'''
Node: node_name

Subscribe to:
    /sub_pose (nav_msgs/Odometry)

Publish to:
    /pub_pose (geometry_msgs/PoseStamped)
'''


import rospy
from nav_msgs.msg import Odometry
from pub_pose.msg import PoseStamped
from std_srvs import Trigger, TriggerRequest, TriggerResponse




class CLASS:
    pass



class ROS_INTERFACE:
    
    def __init__(self):

        # Attributes
        self.odom_data = None
        self.rate = rospy.Rate(10)

        # Initializate node
        rospy.init_node("node_name", anonymous=True)

        # Subscriber
        self.sub = rospy.Subscriber("/sub_topic", Odometry, queue_size=1)
        
        # Publisher
        self.pub = rospy.Publisher("/pub_topic", PoseStamped, odom_callback)
        
        # Service
        # self.serv = rospy.Service('/serv_service', Trigger, trigger_callback)


    def odom_callback(self, msg):
        self.odom_data = msg
    

    # def trigger_callback(self, req):
    #     return TriggerResponse(req.a + req.b)


    def publish(self):

        # Processing data
        pose_data = PoseStamped()
        pose_data.header.stamp = rospy.Time.now()
        pose_data.pose.position.x = self.odom_data.pose.pose.position.x
        ...

        # Publishing
        self.pub.publish(pose_data)
        rospy.loginfo(self.pub)


    def client_service(self, arg):
        rospy.wait_for_service('/cli_service')
        try:
            self.service = rospy.ServiceProxy('/cli_service', Trigger)
            response = self.service(arg)
            return response.string
        except:
            print("Service call failed")


    def main_loop(self):

        while self.odom_data is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Main Loop
        while not rospy.is_shutdown():
            self.publish()
            self.client_service()
            self.rate.sleep()




# def add_two_ints_client(x, y):
#     rospy.wait_for_service('add_two_ints')
#     try:
#         add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
#         resp1 = add_two_ints(x, y)
#         return resp1.sum
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

# def usage():
#     return "%s [x y]"%sys.argv[0]

# if __name__ == "__main__":
#     if len(sys.argv) == 3:
#         x = int(sys.argv[1])
#         y = int(sys.argv[2])
#     else:
#         print(usage())
#         sys.exit(1)
#     print("Requesting %s+%s"%(x, y))
#     print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))



    



if __name__ == '__main__':

    try:
        node = ROS_INTERFACE()
        node.publish()

    except rospy.ROSInterruptException:
        pass