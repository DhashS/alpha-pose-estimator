import rospy

from rssi_dir.msg import Rf_stamped
from geometry_msgs.msg import PoseStamped

rospy.init_node('performance')

from rf_objs import Log_RSSI_sample, Log_odom_sample


def rssi_cb(msg=None):
    rssi = Log_RSSI_sample(msg=msg)
    try:
        rssi.closest_odom = odom_h[-1]
    except IndexError:
	return
    rssi_h.append(rssi)
    #print "appended"
    
def odom_cb(msg=None):
    odom_h.append(Log_odom_sample(msg=msg))

rospy.Subscriber("/RSSI", Rf_stamped, rssi_cb)
rospy.Subscriber('tango_area_pose', PoseStamped, odom_cb)



import numpy as np
from sklearn import gaussian_process as gp
import scipy as sp

rssi_h = []
odom_h = []

perf_logs = {}

class PerformanceTests:
    def __init__(self, tests):
	self.tests = tests
	self.last_res = {}
	while True:
	    self.run_tests()
	
    def run_tests(self):
	for tst in self.tests:
	    tst()
	    try:
	    	if self.last_res[tst.fmt_str] == tst.res:
		    continue
	    except KeyError:
		pass
	    print "{}: {}".format(tst.fmt_str, tst.res)
            self.last_res[tst.fmt_str] = tst.res
	    #self.perf_logs[tst.fmt_str].append(tst.res)

class Test:
    def __init__(self):
	self.fmt_str = None
	self.res = None
	self.f()
	self.res = str(self.res)
	
    def __call__(self):
	return self.__init__()

    def f(self):
	pass

	
class RSSI_L2_mod2(Test):
    def f(self):
	
	rssi_h_copy = rssi_h[:]
	self.fmt_str = "L2 Norm GP error, interleaved"

	get_sigs = lambda x: x.sigs
	get_odom = lambda x: x.closest_odom.pos
	
	class AP_lookup(dict):
        # Globally unique BSSID
            def __init__(self, rssi):
                #get a global list of all AP MAC's seen
		#print "rssi in AP_lookup", rssi
                self.ap_list = map(lambda x: x.sigs, np.ravel(np.array(rssi)))
		#print self.ap_list
                self.ap_list_nodupe = []
                for sig_btch in self.ap_list:
                    for mac, _ in sig_btch:
                        if mac not in self.ap_list_nodupe:
                            self.ap_list_nodupe.append(mac)
		#print self.ap_list_nodupe
                for i, ap in enumerate(self.ap_list_nodupe):
                    self[ap] = i

            def __len__(self):
                return len(self.ap_list_nodupe)
	
	pos = AP_lookup(rssi_h_copy)
	#print len(rssi_h), len(pos)
	rf_space = sp.sparse.lil_matrix((len(rssi_h_copy), len(pos)))
	#print "rf_space shape", rf_space.shape
	for i, sigs in enumerate(map(get_sigs, rssi_h_copy)):
	    for mac, rssi in sigs:
		#print "pos of mac", pos[mac]
		rf_space[i, pos[mac]] = rssi
	odoms = np.zeros((len(rssi_h_copy), 3))
	for i, odom in enumerate(map(get_odom, rssi_h_copy)):
	    odoms[i] = odom

	rssi_h1, rssi_h2 = rf_space[::2], rf_space[1::2]
	odom_h1, odom_h2 = odoms[::2], odoms[1::2]

	
	h1_gp = gp.GaussianProcess(thetaL=0.000001, thetaU=0.002)
	h2_gp = gp.GaussianProcess(thetaL=0.000001, thetaU=0.002)
	
	#print "rssi_h1", rssi_h1.todense()
	#print "odom_h1", odom_h1
	if rssi_h1.shape[0] <= 1:
	   return
	if rssi_h2.shape[0] <= 1:
	   return
	h1_gp.fit(rssi_h1.todense(), odom_h1)
	h2_gp.fit(rssi_h2.todense(), odom_h2)
	err1 = 0
	err2 = 0
	pred1, mse1 = h1_gp.predict(rssi_h2.todense(), eval_MSE=True)
	pred2, mse2 = h2_gp.predict(rssi_h1.todense(), eval_MSE=True)
	err1 = np.sum(np.sqrt(np.sum((np.array(list(map(get_odom, rssi_h_copy[1::2]))) - pred1)**2, 1)))
	err2 = np.sum(np.sqrt(np.sum((np.array(list(map(get_odom, rssi_h_copy[::2]))) - pred2)**2, 1)))
	print "expected valued: {}\n pred1: {}\n trained on: {}\n".format(list(map(get_odom, rssi_h_copy[1::2])), pred1,list(map(get_odom, rssi_h_copy[::2])) )	
	print "h1_gp theta result:", h1_gp.theta_
	self.res = {"err":(err1 + err2)/(len(pred1) + len(pred2)),
		"mse":np.ravel([(m, n) for m, n in zip(mse1, mse2)])}
	return 
		

class RSSI_L2_nextstep(Test):
    def f(self):
	self.fmt_str = "L2 Norm GP error, next step"
	try:
            rssi_h_copy = rssi_h[:]
            rssi_hist, rssi_test = rssi_h_copy[:-1], rssi_h_copy[-1]
	except:
	    return

	if len(rssi_hist) <= 1:
	    return
	if not isinstance(rssi_test, Log_RSSI_sample):
	    return

	get_sigs = lambda x: x.sigs
        get_odom = lambda x: x.closest_odom.pos

        class AP_lookup(dict):
        # Globally unique BSSID
            def __init__(self, rssi):
                #get a global list of all AP MAC's seen
                #print "rssi in AP_lookup", rssi
                self.ap_list = map(lambda x: x.sigs, np.ravel(np.array(rssi)))
                #print self.ap_list
                self.ap_list_nodupe = []
                for sig_btch in self.ap_list:
                    for mac, _ in sig_btch:
                        if mac not in self.ap_list_nodupe:
                            self.ap_list_nodupe.append(mac)
                #print self.ap_list_nodupe
                for i, ap in enumerate(self.ap_list_nodupe):
                    self[ap] = i

            def __len__(self):
                return len(self.ap_list_nodupe)

        pos = AP_lookup(rssi_h_copy)
        #print len(rssi_h), len(pos)
        rf_space = sp.sparse.lil_matrix((len(rssi_hist), len(pos)))
        #print "rf_space shape", rf_space.shape
        for i, sigs in enumerate(map(get_sigs, rssi_hist)):
            for mac, rssi in sigs:
                #print "pos of mac", pos[mac]
                rf_space[i, pos[mac]] = rssi
        odoms = np.zeros((len(rssi_hist), 3))
        for i, odom in enumerate(map(get_odom, rssi_hist)):
            odoms[i] = odom

	rssi_gp = gp.GaussianProcess()

	rssi_gp.fit(rf_space.todense(), odoms)
	
	test = np.zeros(len(pos))
	for sig, rssi in get_sigs(rssi_test):
	    test[pos[sig]] = rssi
	pred, mse = rssi_gp.predict(test, eval_MSE=True)	
	
	err = 0
	err += np.sqrt(np.sum((get_odom(rssi_test) - pred) ** 2))
	self.res = {"err:":err, "mse:":mse}

	return
	
class TestTest(Test):
    def f(self):
	self.fmt_str = "Answer to everything"
	self.res = rssi_h
	return	
	
class RSSI_L2_mod2_orig(Test):
    def f(self):
	
	rssi_h_copy = rssi_h[:]
	self.fmt_str = "L2 Norm GP error, interleaved, no options!"

	get_sigs = lambda x: x.sigs
	get_odom = lambda x: x.closest_odom.pos
	
	class AP_lookup(dict):
        # Globally unique BSSID
            def __init__(self, rssi):
                #get a global list of all AP MAC's seen
		#print "rssi in AP_lookup", rssi
                self.ap_list = map(lambda x: x.sigs, np.ravel(np.array(rssi)))
		#print self.ap_list
                self.ap_list_nodupe = []
                for sig_btch in self.ap_list:
                    for mac, _ in sig_btch:
                        if mac not in self.ap_list_nodupe:
                            self.ap_list_nodupe.append(mac)
		#print self.ap_list_nodupe
                for i, ap in enumerate(self.ap_list_nodupe):
                    self[ap] = i

            def __len__(self):
                return len(self.ap_list_nodupe)
	
	pos = AP_lookup(rssi_h_copy)
	#print len(rssi_h), len(pos)
	rf_space = sp.sparse.lil_matrix((len(rssi_h_copy), len(pos)))
	#print "rf_space shape", rf_space.shape
	for i, sigs in enumerate(map(get_sigs, rssi_h_copy)):
	    for mac, rssi in sigs:
		#print "pos of mac", pos[mac]
		rf_space[i, pos[mac]] = rssi
	odoms = np.zeros((len(rssi_h_copy), 3))
	for i, odom in enumerate(map(get_odom, rssi_h_copy)):
	    odoms[i] = odom

	rssi_h1, rssi_h2 = rf_space[::2], rf_space[1::2]
	odom_h1, odom_h2 = odoms[::2], odoms[1::2]

	
	h1_gp = gp.GaussianProcess()
	h2_gp = gp.GaussianProcess()
	
	#print "rssi_h1", rssi_h1.todense()
	#print "odom_h1", odom_h1
	if rssi_h1.shape[0] <= 1:
	   return
	if rssi_h2.shape[0] <= 1:
	   return
	h1_gp.fit(rssi_h1.todense(), odom_h1)
	h2_gp.fit(rssi_h2.todense(), odom_h2)
	err1 = 0
	err2 = 0
	pred1, mse1 = h1_gp.predict(rssi_h2.todense(), eval_MSE=True)
	pred2, mse2 = h2_gp.predict(rssi_h1.todense(), eval_MSE=True)
	err1 = np.sum(np.sqrt(np.sum((np.array(list(map(get_odom, rssi_h_copy[1::2]))) - pred1)**2, 1)))
	err2 = np.sum(np.sqrt(np.sum((np.array(list(map(get_odom, rssi_h_copy[::2]))) - pred2)**2, 1)))
	#print "expected valued: {}\n pred1: {}\n trained on: {}\n".format(list(map(get_odom, rssi_h_copy[1::2])), pred1,list(map(get_odom, rssi_h_copy[::2])) )	
	print "h1_gp theta result:", h1_gp.theta_
	self.res = {"err":(err1 + err2)/(len(pred1) + len(pred2)),
		"mse":np.ravel([(m, n) for m, n in zip(mse1, mse2)])}
	return 
		




p = PerformanceTests([RSSI_L2_mod2(), RSSI_L2_nextstep()])
rospy.spin()
