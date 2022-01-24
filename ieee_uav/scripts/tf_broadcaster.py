#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
import tf

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class caster():
    def __init__(self):
        rospy.init_node('tf_broadcaster', anonymous=True)
        self.camera_link_name = rospy.get_param("/depth_base", 'd435i/depth_camera_link')
        self.body_link_name = rospy.get_param("/body_base", 'base_link')
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.base_cb)
        self.rate = rospy.Rate(2)

        self.br = tf.TransformBroadcaster()

    def base_cb(self, msg):
        stamp_  = msg.header.stamp
        self.br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),\
(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z, msg.pose.orientation.w),\
stamp_ ,self.body_link_name,"map")
        self.br.sendTransform((0.1, 0.0, -0.05), (0.5,-0.5,0.5,-0.5), stamp_ , self.camera_link_name, self.body_link_name)
        return

if __name__ == '__main__':
    cas = caster()
    while 1:
        try:
            cas.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)