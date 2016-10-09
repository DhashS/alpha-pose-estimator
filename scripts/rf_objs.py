import rospy

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

import math
import time

class RF_direction():

    def __init__(self):
        self.odoms = []
        self.rssi = []

    def log_odom(self, msg):
        self.odoms.append(Log_odom_sample(msg=msg))


    def log_rssi(self, msg):
        self.rssi.append(Log_RSSI_sample(msg=msg))

class Log_odom_sample():
    def __init__(self, msg):
        #self.sec = msg.header.stamp.secs
        #self.nsec = msg.header.stamp.nsecs #TODO left pad
        # gets the heading 
        # TODO Time mgmt
	self.original_msg = msg
        self.sec = rospy.get_rostime()
        self.theta = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]
	self.hdg = self.theta
        # stores the pos
	self.pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) 
        self.x, self.y, self.z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
    def __str__(self):
	return "Log_odom_sample time: {} (x, y, z, theta) = {}, {}, {}, {}".format(self.sec, self.x, self.y, self.z, self.theta)

class Log_RSSI_sample():
        #time skew?
        def __init__(self, msg):
	    self.original_msg = msg
            now = rospy.get_rostime() 
            self.sec = now
            self.nsec = now.nsecs
            self.sigs = self.store_msgs(msg.data)
            self.closest_odom = None

        def store_msgs(self, msg):
            pkt = []
            #there's a some character on the last line that breaks the 
            #BSSID <<mac>> <<dbm>> pattern, so the -1 strips it
            #There's a timestamp in [0]
            #self.secs = msg.split('\n')[0]
            for line in msg.split('\n')[1:-1]:
                _, mac, dbm = line.split(' ')
                pkt.append((mac, int(dbm)))
            return pkt
        def __str__(self):
            return "Log_RSSI_sample: time:{}.{}, signals:{} closest_odom:{}".format\
                                    (self.sec, self.nsec, self.sigs, self.closest_odom)


