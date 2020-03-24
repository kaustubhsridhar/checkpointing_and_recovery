#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from in_loop_2.msg import in_control_2
from out_loop.msg import out_control
from out_loop.msg import anomaly
from coordinator.msg import ckpt_info
from rf_coordinator.msg import check_info
import numpy as np
from safe_auto_nonlinear_base import safe_auto_nonlinear
from in_2_system_description import system_description
 
class inner_controller():
	def __init__(self):
		self.pub = rospy.Publisher('/vL_vals', in_control_2, queue_size=10)
		self.pub_anom = rospy.Publisher('/in_anomaly_L', anomaly, queue_size=10)
		self.wLd = -1;
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
		self.wLd = data.wL

	def publish_to_in(self, event=None):
		ic_msg = in_control_2()
		ic_msg.vL = self.u
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
	sys = safe_auto_nonlinear(sd.func_f, sd.func_A, sd.func_g, sd.func_C, sd.x0, sd.u0, sd.y0, sd.CKPT_INT, sd.T, sd.estimation_method, sd.u_type, sd.devID, sd.Q, sd.R)
	
	r = rospy.Rate(100)
	while sys.NOW<sd.T+0.1:  #see ic.start_time in constructor
		t0O = rospy.get_time()
		RF_time_appended = 0; C_time_appended = 0
		# basic time init (to change)
		sys.NOW = round(rospy.get_time() - ic.start_time, 2); 
		sys.dt = round(sys.NOW - sys.NOW_old,2); 
		if sys.dt == 0: #for simulation (to avoid division by 0)
			continue
		sys.NOW_list.append(sys.NOW); sys.dt_list.append(sys.dt)
		print("(round and not)time is ", sys.NOW, rospy.get_time() - ic.start_time, " and dt is ", sys.dt)
		sys.NOW_old = sys.NOW

		# main code
		ic.check = sys.anomaly_detect()

		# publish to anomaly topic & subscribe to check
		ic.publish_to_anomaly()
		ic.subscribe_to_check_info()
		sys.check = ic.updated_check

		# get estimate and then rf if attack detected
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

		# Create a ROS Timer for reading data at 1/ (A hz)
		ic.subscribe_to_out()
		w_d = np.array([[ic.wLd]])
		
		# control
		sys.u = sys.StartControl(w_d)

		# Create another ROS Timer for publishing data at 1/ (B hz)
		ic.u = sys.u[0,0]
		ic.publish_to_in()

		# Create a ROS Timer for reading data at 1/ (A hz)
		ic.subscribe_to_ckpt_info()
		sys.time_to_CkPt_val = ic.time_to_CkPt_val

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

		dt0O = rospy.get_time() - t0O - dt0C - dt0RF
		sys.other_time_list.append(dt0O)
		r.sleep()

	print("PLOTS UPCOMING")
	sys.plot_curves()

	# Don't forget this or else the program will exit
	rospy.spin()

if __name__ == '__main__':
		main()
