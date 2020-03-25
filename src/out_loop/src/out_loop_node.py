#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from out_loop.msg import out_control
from out_loop.msg import anomaly
#from in_loop_2.msg import in_control_2
#from in_loop.msg import in_control
from coordinator.msg import ckpt_info
from rf_coordinator.msg import check_info
from safe_auto_nonlinear_base import timer_start
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
	safe = safe_auto_nonlinear_ddrive(sd.func_f, sd.func_A, sd.func_g, sd.func_C, sd.x0, sd.u0, sd.y0, sd.CKPT_INT, sd.T, sd.estimation_method, sd.u_type, sd.devID, sd.Q, sd.R)

	r = rospy.Rate(10)
	while safe.NOW<sd.T:
		# basic time init (to change)
		Overall_t = timer_start()
		safe.NOW = round(rospy.get_time() - oc.start_time, 2)
		safe.dt = round(safe.NOW - safe.NOW_old,2)
		if safe.dt == 0: #for simulation (to avoid division by 0)
			continue
		safe.NOW_list.append(safe.NOW); safe.dt_list.append(safe.dt)
		safe.NOW_old = safe.NOW

		# check for anomaly
		oc.check = safe.anomaly_detect()

		# publish to anomaly topic & subscribe to check
		oc.publish_to_anomaly()
		oc.subscribe_to_check_info()
		safe.check = oc.updated_check
		
		# get estimate and then rf if attack detected and then control
		x_estimate = safe.get_x_estimate_from_sensors()
		if safe.check == 0:
			safe.x, safe.y = x_estimate, safe.func_g(x_estimate)
			safe.RF_time_list.append(0)
		elif safe.check == 1:
			RF_t = timer_start()
			safe.x, safe.y = safe.RollForward(x_estimate)
			safe.RF_time_list.append(RF_t.elapsed())
		safe.u = safe.StartControl()

		# transform and publish control
		wR, wL = sd.transform(safe.u)
		oc.u = np.array([[wR],[wL]])
		oc.publish_to_out()

		# subscribe to ckpt_info to get go-ahead to create checkpoint
		oc.subscribe_to_ckpt_info()
		safe.time_to_CkPt_val = oc.time_to_CkPt_val

		# check if time to checkpoint and create checkpoint
		CkPt_check_and_save_t = timer_start()
		if safe.time_to_CkPt_coordinated() == 1:
			safe.create_CkPt()
		safe.CkPt_buffer_time_list.append(CkPt_check_and_save_t.elapsed())

		#oc.subscribe_to_volts()
		safe.other_time_list.append(Overall_t.elapsed() - safe.RF_time_list[-1] - safe.CkPt_buffer_time_list[-1])
		r.sleep()

	print("PLOTS UPCOMING")
	safe.plot_curves_car()

	# Don't forget this or else the program will exit
	rospy.spin()

if __name__ == '__main__':
		main()
