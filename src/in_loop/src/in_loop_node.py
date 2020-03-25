#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from in_loop.msg import in_control
from out_loop.msg import out_control
from out_loop.msg import anomaly
from coordinator.msg import ckpt_info
from rf_coordinator.msg import check_info
import numpy as np
from safe_auto_nonlinear_base import safe_auto_nonlinear, timer_start
from in_system_description import system_description
 
class inner_controller():
	def __init__(self):
		self.pub = rospy.Publisher('/vR_vals', in_control, queue_size=10)
		self.pub_anom = rospy.Publisher('/in_anomaly_R', anomaly, queue_size=10)
		self.wRd = -1;
		self.time_to_CkPt_val = 0
		self.start_time = rospy.get_time() - 0.1 # to avoid divide by dt of 0 in pid
		self.u = np.array([[0],[0]])
		self.check = 0; self.updated_check = 0
	
	def subscribe_to_ckpt_info(self, event=None):
		rospy.Subscriber('/ckpt_info_vals', ckpt_info, self.ckpt_callback)

	def ckpt_callback(self, data):
		self.time_to_CkPt_val = data.time_to_ckpt

	def subscribe_to_check_info(self, event=None):
		rospy.Subscriber('/check_info_vals', check_info, self.check_callback)

	def check_callback(self, data):
		self.updated_check = data.check

	def subscribe_to_out(self, event=None):
		rospy.Subscriber('/out_control_vals', out_control, self.callback)

	def callback(self, data):
		self.wRd = data.wR

	def publish_to_in(self, event=None):
		ic_msg = in_control()
		ic_msg.vR = self.u
		self.pub.publish(ic_msg)

	def publish_to_anomaly(self, event=None):
		ic_msg = anomaly()
		ic_msg.check_anomaly = self.check
		self.pub_anom.publish(ic_msg)


def main():
	rospy.init_node('in_loop_node', anonymous=False) 
	
	# Create instance(s) of class(es)
	sd = system_description()
	ic = inner_controller()
	safe = safe_auto_nonlinear(sd.func_f, sd.func_A, sd.func_g, sd.func_C, sd.x0, sd.u0, sd.y0, sd.CKPT_INT, sd.T, sd.estimation_method, sd.u_type, sd.devID, sd.Q, sd.R)
	
	r = rospy.Rate(100)
	while safe.NOW<sd.T+0.1:  #see ic.start_time in constructor
		# basic time init (to change)
		Overall_t = timer_start()
		safe.NOW = round(rospy.get_time() - ic.start_time, 2)
		safe.dt = round(safe.NOW - safe.NOW_old,2)
		if safe.dt == 0: #for simulation (to avoid division by 0)
			continue
		safe.NOW_list.append(safe.NOW); safe.dt_list.append(safe.dt)
		safe.NOW_old = safe.NOW

		# check for anomaly
		ic.check = safe.anomaly_detect()

		# publish to anomaly topic & subscribe to check
		ic.publish_to_anomaly()
		ic.subscribe_to_check_info()
		safe.check = ic.updated_check

		# get estimate and then rf if attack detected
		x_estimate = safe.get_x_estimate_from_sensors()
		if safe.check == 0:
			safe.x, safe.y = x_estimate, safe.func_g(x_estimate)
			safe.RF_time_list.append(0)
		elif safe.check == 1:
			RF_t = timer_start()
			safe.x, safe.y = safe.RollForward(x_estimate)
			safe.RF_time_list.append(RF_t.elapsed())

		# Subscribe to out_control topic to get reference trajectory for inner controller
		ic.subscribe_to_out()
		w_d = np.array([[ic.wRd]])
		
		# control
		safe.u = safe.StartControl(w_d)

		# publish control
		ic.u = safe.u[0,0]
		ic.publish_to_in()

		# subscribe to ckpt_info to get go-ahead to create checkpoint
		ic.subscribe_to_ckpt_info()
		safe.time_to_CkPt_val = ic.time_to_CkPt_val

		# check if time to checkpoint and create checkpoint
		CkPt_check_and_save_t = timer_start()
		if safe.time_to_CkPt_coordinated() == 1:
			safe.create_CkPt()
		safe.CkPt_buffer_time_list.append(CkPt_check_and_save_t.elapsed())

		safe.other_time_list.append(Overall_t.elapsed() - safe.RF_time_list[-1] - safe.CkPt_buffer_time_list[-1])
		r.sleep()

	print("PLOTS UPCOMING")
	safe.plot_curves()

	# Don't forget this or else the program will exit
	rospy.spin()

if __name__ == '__main__':
		main()
