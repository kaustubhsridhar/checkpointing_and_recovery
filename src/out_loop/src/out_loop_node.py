#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from out_loop.msg import out_control
from out_loop.msg import anomaly
#from in_loop_2.msg import in_control_2
#from in_loop.msg import in_control
from coordinator.msg import ckpt_info
from rf_coordinator.msg import check_info
from safe_auto_nonlinear_ddrive import safe_auto_nonlinear_ddrive
from out_system_description import system_description
import numpy as np
 
class outer_controller():
	def __init__(self):
		self.pub = rospy.Publisher('/out_control_vals', out_control, queue_size=10)
		self.pub_anom = rospy.Publisher('/out_anomaly', anomaly, queue_size=10)
		self.time_to_CkPt_val = 0
		self.start_time = rospy.get_time()
		self.u = np.array([[0],[0]])

		#self.vR = -1; self.vL = -1
		self.check = 0; self.updated_check = 0
	
	def subscribe_to_ckpt_info(self, event=None):
		rospy.Subscriber('/ckpt_info_vals', ckpt_info, self.ckpt_callback)

	def ckpt_callback(self, data):
		self.time_to_CkPt_val = data.time_to_ckpt

	def subscribe_to_check_info(self, event=None):
		rospy.Subscriber('/check_info_vals', check_info, self.check_callback)

	def check_callback(self, data):
		self.updated_check = data.check

	#def subscribe_to_volts(self, event=None):
	#	#s_str = "-------------------------- %s" % rospy.get_time()
	#	#rospy.loginfo(s_str)
	#	rospy.Subscriber('/vR_vals', in_control, self.vR_callback)
	#	rospy.Subscriber('/vL_vals', in_control_2, self.vL_callback)

	#def vR_callback(self, data):
	#	self.vR = data.vR

	#def vL_callback(self, data):
	#	self.vL = data.vL

	def publish_to_out(self, event=None):
		oc_msg = out_control()
		oc_msg.wR = self.u[0,0]
		oc_msg.wL = self.u[1,0]
		self.pub.publish(oc_msg)

	def publish_to_anomaly(self, event=None):
		oc_msg = anomaly()
		oc_msg.check_anomaly = self.check
		self.pub_anom.publish(oc_msg)


def main():
	rospy.init_node('out_loop_node', anonymous=False)

	# Create instance(s) of class(es)
	sd = system_description()
	oc = outer_controller()
	sys = safe_auto_nonlinear_ddrive(sd.func_f, sd.func_A, sd.func_g, sd.func_C, sd.x0, sd.u0, sd.y0, sd.CKPT_INT, sd.T, sd.estimation_method, sd.u_type, sd.devID, sd.Q, sd.R)

	r = rospy.Rate(10)
	while sys.NOW<sd.T:
		t0O = rospy.get_time()
		RF_time_appended = 0; C_time_appended = 0
		# basic time init (to change)
		sys.NOW = round(rospy.get_time() - oc.start_time, 2); 
		sys.dt = round(sys.NOW - sys.NOW_old,2); 
		if sys.dt == 0: #for simulation (to avoid division by 0)
			continue
		sys.NOW_list.append(sys.NOW); sys.dt_list.append(sys.dt)
		print("(round and not)time is ", sys.NOW, rospy.get_time() - oc.start_time, " and dt is ", sys.dt)
		sys.NOW_old = sys.NOW

		# main code
		oc.check = sys.anomaly_detect()

		# publish to anomaly topic & subscribe to check
		oc.publish_to_anomaly()
		oc.subscribe_to_check_info()
		sys.check = oc.updated_check
		
		# get estimate and then rf if attack detected and then control
		x_estimate = sys.get_x_estimate_from_sensors()
		if sys.check == 0:
			sys.x, sys.y = x_estimate, sys.func_g(x_estimate)
			sys.rf_count = 0 # reset to 0 for next rf to start from ckpt
		elif sys.check == 1:
			t0RF = rospy.get_time()
			sys.x, sys.y = sys.RollForward(x_estimate)
			dt0RF = rospy.get_time() - t0RF
			sys.RF_time_list.append(dt0RF)
			RF_time_appended = 1
		sys.u = sys.StartControl()

		# transform and publish control
		wR, wL = sd.transform(sys.u)
		oc.u = np.array([[wR],[wL]])
		oc.publish_to_out()

		# subscribe to ckpt time
		oc.subscribe_to_ckpt_info()
		sys.time_to_CkPt_val = oc.time_to_CkPt_val

		# create checkpoint
		if sys.time_to_CkPt_coordinated() == 1:
			t0C = rospy.get_time()
			sys.create_CkPt()
			dt0C = rospy.get_time()-t0C
			sys.CkPt_buffer_time_list.append(dt0C)
			C_time_appended = 1

		if C_time_appended == 0:
			sys.CkPt_buffer_time_list.append(0); dt0C = 0
		if RF_time_appended == 0:
			sys.RF_time_list.append(0); dt0RF = 0

		#oc.subscribe_to_volts()
		dt0O = rospy.get_time() - t0O - dt0C - dt0RF
		sys.other_time_list.append(dt0O)
		r.sleep()

	print("PLOTS UPCOMING")
	sys.plot_curves_car()

	# Don't forget this or else the program will exit
	rospy.spin()

if __name__ == '__main__':
		main()
