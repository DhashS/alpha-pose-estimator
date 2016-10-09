#!/usr/bin/env python

import rospy
import socket
import fcntl
import os
import pickle

from sklearn import gaussian_process

from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String, Header

from rssi_dir.msg import Rf_stamped

import rf_objs
	
#print "Starting socket init"
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
fcntl.fcntl(sock, fcntl.F_SETFL, os.O_NONBLOCK)
if os.path.exists("../RSSI_sock"):
    os.remove("../RSSI_sock")
sock.bind("../RSSI_sock")
sock.listen(1)
conn = None
cli = None
#print "Socket init finished"

if True: #__name__ == "__main__":
    rospy.init_node('look_rf')
    print "logger & history server init"
    prg = rf_objs.RF_direction()
    #sensors
    rospy.Subscriber('tango_area_pose', PoseStamped, prg.log_odom)
    rospy.Subscriber('/RSSI', Rf_stamped, prg.log_rssi)

    r = rospy.Rate(10)
    while True:
        try:
    	    conn, cli = sock.accept()
	    fcntl.fcntl(conn, fcntl.F_SETFL, os.O_NONBLOCK)
    	    print "History server connected"
	    break
        except:
            pass


    while not rospy.is_shutdown():
	try:
	    req = conn.recv(4096)  
            if req == b"req-dump":
		data = ""
		#print "dumping objs"
                conn.sendall(b"odom-start")
		data += "odom-start"
                conn.sendall(pickle.dumps(prg.odoms))
		data += pickle.dumps(prg.odoms)
                conn.sendall(b"odom-end")
                conn.sendall(b"rssi-start")
		data += "odom-endrssi-start"
                conn.sendall(pickle.dumps(prg.rssi))
		data += pickle.dumps(prg.rssi)
                conn.sendall(b"rssi-end")
		data += "rssi-end"
		print "outbound data is {} long".format(len(data))
		#print "dumping objects done"
	except:
	    pass
        r.sleep()
