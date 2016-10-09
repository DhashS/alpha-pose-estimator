import rospy

from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix

from rssi_dir.msg import Rf_stamped
import rssi_dir.srv as srvs
from rf_objs import *


class Particle:
    """Represents a hypothesis of where the tango is, in x, y, z, and theta (yaw angle)
       as well as a probability that the current hypothesis is correct
        x: x-coord relative to global map frame
        y: y-coord relative to global map frame
        z: z-coord relative to global map frame
        theta: angle relative to global map frame (xy-plane, +x, 0y is 0\degree)
    """
    def __init__(self, x=0.0, y=0.0, z=0.0, theta=0.0):
        #Pose data
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta

    def as_pose(self):
        """Converted to a ROS pose message"""
        as_quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=self.y),\
                    orientation=Quaternion(x=as_quaternion[0], y=as_quaternion[1],\
                                           z=as_quaternion[2], w=as_quaternion[3]))

class ParticleWrapper:
    """A class that wraps all pose-predictors and stores domain-specific information about
       them for each particle
       particle: the particle
       frame: what coordinate frame all the poses are in
       ts: timestamp
       p: probability of particle hypothesis
       mu: probability curve center
       variance: variance at timestep
    """
    def __init__(self, particle=None, frame="device", ts=rospy.Time.now(), p=1.0, mu=0.0, variance=1.0):
        self.particle = particle
        self.frame = frame
        self.ts = ts

        #probabililty
        self.p = p
        self.mu = mu
        self.variance = variance

    def as_pose(self):
        """Converted to a ROS pose message"""
        return self.particle.as_pose()


class GPhandler:
    """Defines a wrapper class around handling interfacing with the gaussian process
       service_name: name of the GP rospy synchronous service
    """
    def __init__(self, service_name=""):
            rospy.wait_for_service('predict_rf')
            rospy.wait_for_service('predict_odom')
            rospy.wait_for_service('compute')
            self.compute_h = rospy.ServiceProxy('compute', srvs.compute)
            self.forward_h = rospy.ServiceProxy('predict_rf', srvs.odom_gp)
            self.backward_h = rospy.ServiceProxy('predict_odom', srvs.rssi_gp)
            self.train()
    def ask(self, arg):
        #print "GPHandler: type of req: {}, RSSI:{}, odom:{}".format(type(arg), isinstance(arg, Log_RSSI_sample), isinstance(arg, Log_odom_sample))
        if isinstance(arg, Log_odom_sample):
            #print "Processing Odom update"
            #print arg.original_msg
            resp = self.forward_h(arg.original_msg)
            return resp
        if isinstance(arg, Log_RSSI_sample):
            resp = self.backward_h((arg.original_msg))
            return resp

    def train(self):
        self.compute_h(None)

