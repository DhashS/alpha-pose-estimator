#!/usr/bin/env python

import socket
import os
import fcntl
import sys
import rospy
import pickle

from sklearn import gaussian_process

from rf_objs import Log_RSSI_sample, Log_odom_sample

rf_to_odom = None
odom_to_rf = None
ap_id = {}
ap_id_rev = {}

sock_loc = "../RSSI_sock"
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
fcntl.fcntl(sock, fcntl.F_SETFL, os.O_NONBLOCK)
sock.connect(sock_loc)

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


        gpf.fit(odom, rf_sigs)
        gpb.fit(rf_sigs, odom)

        return (gpf, gpb)


    ap_id = AP_lookup(rssi)
    ap_id_rev = {v: k for k, v in ap_id.items()}
    train = gen_rf_space(dirs,rssi)
    odom_to_sig, sig_to_odom = gaussian_proc(train)
    return(odom_to_sig, sig_to_odom)

    
def recv_data():
	global sock

        sock.send(b"req-dump")
	rssi_h = None
	odom_h = None
	data = None
	#Sleep here for data if not working
	while True:
	    try:
	        data += sock.recv(4096)
	    except:
		break

	rssi_h = pickle.loads(data[data.find('rssi-start')+len("rssi-start"):data.find("rssi-end")])
	odom_h = pickle.loads(data[data.find('odom-start')+len("odom-start"):data.find("odom-end")])
        construct(odom_h, rssi_h)
 
        


def predict_rf(msg):

    global ap_id_rev
    global forward

    point = Log_odom_sample(msg=msg)
    all_data = list(point.pos) + [point.hdg]
    pred, MSE = forward.predict(all_data, eval_MSE=True)
    return (str(pred), str(MSE))

def predict_loc(msg):

    global ap_id
    global backward

    rf = Log_RSSI_sample(msg=msg)
    sigs = np.zeros((len(rf.sigs)))

    for mac, dbm in rf.sigs:
        sigs[ap_id[mac]] = dbm

    pred, MSE = backward.predict(sigs, eval_MSE=True)
    return (str(pred), str(MSE))

def pred_server():
    rospy.init_node("odom_rf_prediction_server")
    recv_data()
    compute_service = rospy.Service('compute', recv_data)
    forward_predict_service = rospy.Service('predict_rf', predict_rf)
    backward_predict_service = rospy.Service('predict_odom', predict_odom)
    

if __name__ == "__main__":
    pred_server()

    
    
