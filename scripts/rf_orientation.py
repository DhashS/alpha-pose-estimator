#!/usr/bin/env python

import rospy
import socket
import fcntl
import os

from sklearn import gaussian_process

from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String, Header

import rf_objs
	
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
fcntl.fcntl(sock, fcntl.F_SETFL, os.O_NONBLOCK)
if os.path.exists("/home/dhash/research/ROS/src/rssi_dir/RSSI_sock"):
    os.remove("/home/dhash/research/ROS/src/rssi_dir/RSSI_sock")
sock.bind("/home/dhash/research/ROS/src/rssi_dir/RSSI_sock")
sock.listen(1)
conn = None
cli = None

if __name__ == "__main__":
    rospy.init_node('look_rf')
    
    prg = rf_objs.RF_direction()
    #sensors
    rospy.Subscriber('tango_area_pose', PoseStamped, prg.log_odom)
    rospy.Subscriber('/RSSI', String, prg.log_rssi)

    r = rospy.Rate(10)
    while True:
        try:
    	    conn, cli = socket.accept()
	    fcntl.fcntl(conn, fcntl.F_SETFL, os.O_NONBLOCK)
    	    print "History server connected"
        except:
            pass


    while not rospy.is_shutdown():
	try:
	    req = conn.recv(4096) 
            if req == b"req-dump":
                conn.send(b"odom-start")
           	conn.send(pickle.dumps(prg.odoms))
           	conn.send(b"odom end")
           	conn.send(b"rssi-start")
                conn.send(pickle.dumps(prg.rssi))
           	conn.send(b"rssi-end")
	except BlockingIOError:
	    pass
        r.sleep()
