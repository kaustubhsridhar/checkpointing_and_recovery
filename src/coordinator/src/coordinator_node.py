#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from coordinator.msg import ckpt_info
 
class coordinator():
	def __init__(self):
		self.pub = rospy.Publisher('/ckpt_info_vals', ckpt_info, queue_size=10)
		self.time_to_ckpt = 0
		self.count = 0

	def publish_to_ckpt_info(self, event=None):
		msg = ckpt_info()
		if self.count%10 == 0:
			msg.time_to_ckpt = 1
		else:
			msg.time_to_ckpt = 0
		self.count += 1
		
		p_str = "publishing now at %s" % rospy.get_time()
		rospy.loginfo(p_str)
		self.pub.publish(msg)


def main():
	rospy.init_node('coordinator_node', anonymous=False)

	# Create an instance of coordinator class
	co = coordinator()

	r = rospy.Rate(10)
	# main code
	while not rospy.is_shutdown():
		co.publish_to_ckpt_info()
		r.sleep()

	# Don't forget this or else the program will exit
	rospy.spin()

if __name__ == '__main__':
		main()
