#!/usr/bin/env python

import socket
import os
import fcntl
import sys
import rospy
import pickle
import time

from sklearn import gaussian_process

from rf_objs import Log_RSSI_sample, Log_odom_sample
import numpy as np
import scipy as sp

import rssi_dir.srv as srvs


time.sleep(10)
rf_to_odom = None
odom_to_rf = None
ap_id = {}
ap_id_rev = {}

sock_loc = "../RSSI_sock"
#block until the socket is established
while not os.path.exists(sock_loc):
    pass



sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#fcntl.fcntl(sock, fcntl.F_SETFL, os.O_NONBLOCK)
#print "Service init complete: starting services"
while True:
    try:
	sock.connect(sock_loc)
	break
    except:
	pass

def construct(dirs, rssi):

    global ap_id
    global ap_id_rev
    global rf_to_odom 
    global odom_to_rf

    class AP_lookup(dict):
    # Globally unique BSSID
        def __init__(self, rssi):
            #get a global list of all AP MAC's seen
            self.ap_list = map(lambda x: x.sigs, np.ravel(np.array(rssi)))
            self.ap_list_nodupe = []
            for sig_btch in self.ap_list:
                for mac, _ in sig_btch:
                    if mac not in self.ap_list_nodupe:
                        self.ap_list_nodupe.append(mac)
            for i, ap in enumerate(self.ap_list_nodupe):
                self[ap] = i

        def __len__(self):
            return len(self.ap_list_nodupe)

    def gen_rf_space(dirs, rssi):
        def get_closest_odom(loc):
            return list(dirs[loc].pos) + [dirs[loc].hdg]
        #pack into a sparse matrix, each one a signal stringth of a MAC
        odom_times = [x.sec for x in dirs][::-1]
        for i, sig_btch in enumerate(rssi):
            if rssi[i].closest_odom != None:
                continue
            rf_space = sp.sparse.lil_matrix((len(rssi), len(ap_id)))
            for mac, dbm in sig_btch.sigs:
                rf_space[i, ap_id[mac]] = dbm
            rssi[i].sigs = rf_space
            last_direction = None
            pos = 0
	    #print "time types: {} {}".format(type(odom_times[pos]), type(rssi[i].sec))
            while odom_times[pos] > rssi[i].sec:
                pos += 1
            rssi[i].closest_odom = get_closest_odom(-1 * pos)

        return rssi

    def gaussian_proc(train):
        gpf = gaussian_process.GaussianProcess(theta0=1e-2, thetaL=1e-4, thetaU=1e-1)
        gpb = gaussian_process.GaussianProcess(theta0=1e-2, thetaL=1e-4, thetaU=1e-1)


        odom = map(lambda x: x.closest_odom, train)
        rf_sigs = reduce(lambda x, y: x+y, map(lambda x: x.sigs, train)).todense()
        rf_sigs = np.array(rf_sigs)
        odom = np.array(odom)

	#print odom
	#print rf_sigs
        gpf.fit(odom, rf_sigs)
        gpb.fit(rf_sigs, odom)

        return (gpf, gpb)


    ap_id = AP_lookup(rssi)
    ap_id_rev = {v: k for k, v in ap_id.items()}
    train = gen_rf_space(dirs,rssi)
    odom_to_rf, rf_to_odom = gaussian_proc(train)
    
    return(odom_to_rf, rf_to_odom)

    
def recv_data(msg=None):
	global sock

        sock.send("req-dump")
	rssi_h = None
	odom_h = None
	data = ""
	#Sleep here for data if not working
	#print "prepping to recv objects"
	#time.sleep(5)
	while True:
	    #try:
	    data += sock.recv(4096)
	    #print "data recvd"
	    #keep recving until transmission over
	    #print "len data: {}".format(len(data))
	    if data[-len("rssi-end"):] == "rssi-end":
	        #print "history data recieved"
		break
	    #except:
		#print sys.exc_info()[0]

	#print "objects recv, len: {}".format(len(data))


	#print "odom : {} -> {}".format(data.find('odom-start')+len('odom-start'),data.find('odom-end'))
	#print "odom first and last bytes: {} : {}".format(data[data.find('odom-start')+len('odom-start')-10:data.find('odom-start')+len('odom-start') + 10], data[data.find('rssi-end')-10:])
        #print "rssi : {} -> {}".format(data.find('rssi-start')+len('rssi-start'),data.find('rssi-end'))
        print "The data is {} long".format(len(data))
	odom_h = pickle.loads(data[data.find('odom-start')+len('odom-start'):data.find('odom-end')])
	rssi_h = pickle.loads(data[data.find('rssi-start')+len("rssi-start"):data.find("rssi-end")])
	#print "unpickled"
	print "Odom data: {} \nRSSI data: {}\n".format(map(lambda x: x.pos, odom_h), map(lambda x: x.sigs, rssi_h))

        construct(odom_h, rssi_h)
	return "Complete" 
        


def predict_rf(msg):
    #print "Running RSSI GP"
    global ap_id_rev
    global odom_to_rf


    point = Log_odom_sample(msg=msg.indata)
    all_data = list(point.pos) + [point.hdg]
    #print "trying to predict {}".format(all_data)
    pred, MSE = odom_to_rf.predict(all_data, eval_MSE=True)
    #print "GP run, pred:{}, MSE:{}".format(pred, MSE)
    return {"rssi":pred[0], "mse":MSE}

def predict_loc(msg):
    print "Running Odom GP"
    global ap_id
    global rf_to_odom

    rf = Log_RSSI_sample(msg=msg.indata)
    sigs = np.zeros(len(ap_id)) 
    for mac, dbm in rf.sigs:
	try:
	    col = ap_id[mac]        
	    sigs[col] = dbm
	except KeyError:
		pass

    pred, MSE = rf_to_odom.predict(sigs, eval_MSE=True)
    #print "GP run, pred:{}, MSE:{}".format(pred, MSE)
    pred = map(lambda x: float(x), list(pred[0]))
    return {"pose":list(pred), "mse":MSE}

def pred_server():
    rospy.init_node("odom_rf_prediction_server")
    print "Node init, getting data"
    recv_data()
    print "Initial compute run, startig services"
    compute_service = rospy.Service('compute', srvs.compute, recv_data)
    forward_predict_service = rospy.Service('predict_rf', srvs.odom_gp, predict_rf)
    backward_predict_service = rospy.Service('predict_odom', srvs.rssi_gp, predict_loc)
    print "All services started!"
    rospy.spin()

if __name__ == "__main__":
    pred_server()

    
    
