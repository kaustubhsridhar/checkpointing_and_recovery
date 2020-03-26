#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from rf_coordinator.msg import check_info
from out_loop.msg import anomaly
 
class coordinator():
	def __init__(self):
		self.pub = rospy.Publisher('/check_info_vals', check_info, queue_size=10)
		self.check_out = 0; self.check_in_R = 0; self.check_in_L = 0

	def publish_to_ckpt_info(self, event=None):
		msg = check_info()
		if self.check_out == 1 or self.check_in_R == 1 or self.check_in_L == 1:
			msg.check = 1
		else:
			msg.check = 0

		#p_str = "each check now at %s" % rospy.get_time()
		#rospy.loginfo(p_str)
		#print(self.check_out, self.check_in_R, self.check_in_L, msg.check)
		self.pub.publish(msg)

	def subscribe_to_out_anom(self, event=None):
		rospy.Subscriber('/out_anomaly', anomaly, self.out_anom_callback)
	def out_anom_callback(self, data):
		self.check_out = data.check_anomaly	

	def subscribe_to_in_R_anom(self, event=None):
		rospy.Subscriber('/in_anomaly_R', anomaly, self.in_R_anom_callback)
	def in_R_anom_callback(self, data):
		self.check_in_R = data.check_anomaly

	def subscribe_to_in_L_anom(self, event=None):
		rospy.Subscriber('/in_anomaly_L', anomaly, self.in_L_anom_callback)
	def in_L_anom_callback(self, data):
		self.check_in_L = data.check_anomaly	


def main():
	rospy.init_node('RF_coordinator_node', anonymous=False)

	# Create an instance of coordinator class
	co = coordinator()

	r = rospy.Rate(100)
	# main code
	while not rospy.is_shutdown():
		co.subscribe_to_out_anom()
		co.subscribe_to_in_R_anom()
		co.subscribe_to_in_L_anom()

		co.publish_to_ckpt_info()
		r.sleep()

	# Don't forget this or else the program will exit
	rospy.spin()

if __name__ == '__main__':
		main()
