#!/usr/bin/env python


import rospy

rospy.init_node('alphaposeestimator')
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

import random
import itertools
import numpy as np

from tf_helpers import TransformHelpers
from pf_objs import Particle, ParticleWrapper, GPhandler


class ParticleFilter:
    """Represents a particle filter, responsible for updating Particle positions and
       probabilities
    """
    def __init__(self):
        #rospy.init_node('ParticleFilter')
        #rospy.wait_for_service('RSSI_GP')
        
        self.map_frame = "map"
        self.area_learning_frame = "area_learning"
        self.base_frame = "device"

        self.gp = GPhandler()

        self.num_particles_derived = 10
        self.theta_hypothesis = 4
        self.particle_list = []
        self.alpha_pose = None

	self.stamp = None
        self.last_reading = {}

	self.unprocessed_poses = []       
 
	self.tf_listener = TransformListener()
	self.tf_broadcaster = TransformBroadcaster()
	rospy.Subscriber("/RSSI", Rf_stamped, self.recv_RSSI_pose)
	rospy.Subscriber("/tango_area_pose", PoseStamped, self.recv_area_learning_pose)
	self.cloud_publisher = rospy.Publisher("/particle_cloud", PoseArray, queue_size=10)
	self.rssi_pose = rospy.Publisher("/rssi_gp_pose", PoseStamped, queue_size=10)

    #Core logic
    def init_particles(self, shape="cube", size=1, pos=None, sigma=None, mu=None):
        

	if pos == None:
	    pos = Particle(x=0,y=0,z=0,theta=0)

	if shape == "cube":
            xs = np.linspace(pos.x - size, pos.x + size, num=self.num_particles_derived)
            ys = np.linspace(pos.y - size, pos.y + size, num=self.num_particles_derived)
            zs = np.linspace(pos.z - size, pos.z + size, num=self.num_particles_derived)
            thetas = np.linspace(0, 360, num=self.theta_hypothesis)
            for xyzt in itertools.product(xs, ys, zs, thetas):
                self.particle_list.append(ParticleWrapper(particle=Particle(x=xyzt[0],\
                                                                        y=xyzt[1],\
                                                                        z=xyzt[2],\
                                                                        theta=xyzt[3]),\
                                                          frame=self.base_frame,\
							  ts=rospy.Time.now(),\
                                                          p=(1.0/self.num_particles_derived ** 3 * self.theta_hypothesis)\
                                                          )) #TODO: get this from pos

        if shape == "gaussian":
            for i in range(0, self.num_particles_derived):
                self.particle_list.append(ParticleWrapper(particle=Particle(x=(pos.x + random.gauss(mu, sigma)),\
                                                                        y=(pos.y + random.gauss(mu, sigma)),\
                                                                        z=(pos.z + random.gauss(mu, sigma)),\
                                                          theta=(pos.theta + random.gauss(mu, sigma))),\
                                                          frame=self.base_frame,\
	                                                  ts=rospy.Time.now(),\
              		                                  p=(1.0/self.num_particles_derived))) #TODO: get this from pos

        self.normalize_particles()
    
    def alpha_pose_eval(self, method="avg", fallback="mode"):
        if method == "avg":
            avg = np.zeros(4)
            for pwrap in self.particle_list:
                avg[0] += pwrap.particle.x
                avg[1] += pwrap.particle.y
                avg[2] += pwrap.particle.z
                avg[3] += pwrap.particle.theta
            avg /= len(self.particle_list)
            self.alpha_pose = Particle(x=avg[0], y=avg[1], z=avg[2], theta=avg[3]).as_pose()
        if method == "mode":
            #TODO
            pass
        if method == "prob":
            #TODO
            pass
    def update_particles(self, opt_dicts=[]):
        self.normalize_particles()
        for p in self.unprocessed_poses:
            """new reading to particle cloud update funcs"""
            if isinstance(p, Log_RSSI_sample):
                #Get speed, and therefore expected_distance, angle, rotational velocity
                opts = opt_dicts[0]
                resp = self.gp.ask(p)
		#print "got GP response"
		loc, err = resp.pose, resp.mse
		print "RSSI GP Result: err:{} loc:{}".format(err, loc)
		as_quaternion = tf.transformations.quaternion_from_euler(0, 0, loc[3])
		self.rssi_pose.publish(PoseStamped(header=Header(stamp=p.original_msg.header.stamp, frame_id=p.original_msg.header.frame_id), pose=Pose(Point(x=loc[0], y=loc[1], z=loc[2]), Quaternion(x=as_quaternion[0],y=as_quaternion[1],z=as_quaternion[2],w=as_quaternion[3]))))
		
		
                for pw in self.particle_list:
		    #discuss with paul
		    #TODO: particle update :)
                    pw.p *= err * opts["RSSI_acc"] * opts["RSSI_dist_falloff"] * np.sqrt((loc[0] - pw.particle.x) ** 2 + (loc[1] - pw.particle.y) ** 2)

                self.normalize_particles()

            if isinstance(p, Log_odom_sample):
                opts = opt_dicts[1]
		#resp = self.gp.ask(p)
		#print "GOt GP response"
		#print resp
		#rssi, err = resp.rssi, resp.mse
		delta_rot1, delta_rot2, delta_trans = None, None, None
                try: 
                    delta_rot1 = ParticleFilter.angle_diff(math.atan2(p.x - self.last_reading["Log_odom"].x,\
                                                                       p.y - self.last_reading["Log_odom"].y),\
                                                            	       p.theta)
                    delta_rot2 = ParticleFilter.angle_diff(p.theta - self.last_reading["Log_odom"].theta, delta_rot_1)
                    delta_trans = np.sqrt((p.x - self.last_reading["Log_odom"].x)**2 + (p.y - self.last_reading["Log_odom"].y)**2)
		    self.last_reading["Log_odom"] = p
                except KeyError:
                    delta_rot1 = math.atan2(p.x, p.y)
                    delta_rot2 = p.theta
		    delta_trans = 1.0

		try:
		    self.last_reading["Log_odom_drift"] += opts["odom_drift"]	
		except KeyError:
		    self.last_reading["Log_odom_drift"] = 0.01


                delta_rot1_noise = min(math.fabs(ParticleFilter.angle_diff(delta_rot1, 0.0)), math.fabs(ParticleFilter.angle_diff(delta_rot1, math.pi)))
                delta_rot2_noise = min(math.fabs(ParticleFilter.angle_diff(delta_rot2, 0.0)), math.fabs(ParticleFilter.angle_diff(delta_rot2, math.pi)))

                for pw in self.particle_list:
                    # Sample pose differences
                    delta_rot1_hat = ParticleFilter.angle_diff(delta_rot1, gauss(0, opts["alpha1"]*delta_rot1_noise*\
                            delta_rot1_noise + opts["alpha2"]*delta_trans*delta_trans))
                    delta_trans_hat = delta_trans - gauss(0, opts["alpha3"]*delta_trans*delta_trans +\
                            opts["alpha4"]*delta_rot1_noise*delta_rot1_noise + opts["alpha4"]*delta_rot2_noise*delta_rot2_noise)
                    delta_rot2_hat = ParticleFilter.angle_diff(delta_rot2, gauss(0, opts["alpha1"]*delta_rot2_noise*delta_rot2_noise\
                            + opts["alpha2"]*delta_trans*delta_trans))
                    # Apply sampled update to particle pose
                    pw.particle.x += delta_trans_hat * math.cos(pw.particle.theta + delta_rot1_hat)
                    pw.particle.y += delta_trans_hat * math.sin(pw.particle.theta + delta_rot1_hat)
                    pw.particle.theta += delta_rot1_hat + delta_rot2_hat
		    pw.p += random.gauss(0, self.last_reading["Log_odom_drift"] * delta_trans)

    #Helper functions
    def normalize_particles(self):
        prob_sum = sum(map(lambda x: x.p, self.particle_list))
        for i in range(0, len(self.particle_list)):
            self.particle_list[i].p /= prob_sum
    
    #ROS I/O
    def publish_particles(self, msg=None):
        #print "publishing particles"
	particle_poses = [x.as_pose() for x in self.particle_list]
	#print "num particles:{}".format(len(particle_poses))
        cloud = PoseArray(header=Header(stamp=self.stamp, frame_id="area_learning"),
		         poses=particle_poses)
	self.cloud_publisher.publish(cloud)

    #ROS callbacks
    #If we're running this online these get hit
    def recv_RSSI_pose(self, msg):
	pose = Log_RSSI_sample(msg)
        self.unprocessed_poses.append(pose)

    def recv_area_learning_pose(self, msg):
        pose = Log_odom_sample(msg)
	self.last_area_learning_pose = pose 
        #Do a wrap in a
	self.stamp = msg.header.stamp  
        self.unprocessed_poses.append(pose)
	#print "Odom update ingest"

    @staticmethod
    def angle_normalize(z):
   	""" convenience function to map an angle to the range [-pi,pi] """
   	return math.atan2(math.sin(z), math.cos(z))

    @staticmethod
    def angle_diff(a, b):
	""" Calculates the difference between angle a and angle b (both should be in radians)
		the difference is always based on the closest rotation from angle a to angle b
		examples:
			angle_diff(.1,.2) -> -.1
			angle_diff(.1, 2*math.pi - .1) -> .2
			angle_diff(.1, .2+2*math.pi) -> -.1
	"""
	a = ParticleFilter.angle_normalize(a)
	b = ParticleFilter.angle_normalize(b)
	d1 = a-b
	d2 = 2*math.pi - math.fabs(d1)
	if d1 > 0:
            d2 *= -1.0
	if math.fabs(d1) < math.fabs(d2):
	    return d1
	else:
	    return d2

    def run(self):
	#Block until we can initialize particles with the first unprocessed_pose
	while True:
	    try:
		fst = self.unprocessed_poses[0]
		if isinstance(fst, Log_odom_sample):
		    self.init_particles(pos=fst)
		    break
		else:
		    pass
	    except IndexError:
		pass
		    
	while True:
	    self.gp.train() #Thread this one out for non-synchronous
			    #I.e if we don't want to train a GP every time
	    self.normalize_particles()  
		
	    update_options = [{"RSSI_acc": 1, "RSSI_dist_falloff": 1, "odom_drift": 0.01}, {"alpha1": 0.2, "alpha2": 0.2, "alpha3": 0.2, "alpha4": 0.2}]
	
	    self.update_particles(opt_dicts=update_options)
            self.alpha_pose_eval()
#	    self.device_map_frame_transform()
	    self.publish_particles()
	
pf = ParticleFilter()

pf.run()
    

