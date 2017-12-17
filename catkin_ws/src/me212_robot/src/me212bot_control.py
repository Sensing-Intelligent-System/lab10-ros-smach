#!/usr/bin/python

import rospy
import tf
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist
import math
from std_msgs.msg import Float64MultiArray
from me212_robot.msg import smachAction, smachResult, smachFeedback
import actionlib

class me212bot_control(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.br = tf.TransformBroadcaster()
		self.x = 0 # robot position x in meter
		self.y = 0 # robot position y in meter
		self.yaw = 0 # robot pose theta in radian
		self.roll = 0
		self.pitch = 0
		self.frame_id = 0

		# publisher
		self.odm_pub = rospy.Publisher('odometry', Float64MultiArray, queue_size=10, latch=True)

		# subsrciber
		self.sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.cbPose)


		self.smach_action = actionlib.SimpleActionServer("smach_action", smachAction, execute_cb=self.actionCB, auto_start = False)
		self.smach_feedback = smachFeedback()
		self.smach_result = smachResult()
		self.smach_action.start()


		rospy.on_shutdown(self.custom_shutdown) # shutdown method
		rospy.loginfo("[%s] Initialized " %self.node_name)
		self.now = rospy.get_time() # start

	def custom_shutdown(self):
		rospy.loginfo("[%s] Shutting down..." %self.node_name)
 

	def cbPose(self, msg):
		if self.smach_action.is_active():
			return
			
		self.x += msg.linear.x * math.cos(self.yaw) * 0.01
		self.y += msg.linear.x * math.sin(self.yaw) * 0.01
		self.yaw += msg.angular.z * 0.01
		self.pub_frame()


	def actionCB(self, goal):
		if goal.action_id ==0:
			self.go_straight(goal.number)
		elif goal.action_id ==1:
			self.go_circle(goal.number)
       
		rospy.loginfo('smach_action: Succeeded')
		self.smach_action.set_succeeded()

	def go_straight(self, distance):
		
		for x in range(100):
			self.x += math.cos(self.yaw) * distance * 0.01 * 0.01
			self.y += math.sin(self.yaw) * distance * 0.01 * 0.01
			self.pub_frame()
			rospy.sleep(0.01)

	def go_circle(self, number):
		total = math.pi * 2 * number + self.yaw

		while (total - self.yaw) > 0 :
			self.yaw += 0.001
			self.pub_frame()
			rospy.sleep(0.001)



	def pub_frame(self):
		self.br.sendTransform((self.x, self.y, 0), # to 3d translation
                              tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw), # to 3d rotation
                              rospy.Time.now(), # timestamp
                              "base_link", # robot frame
                              "world") # base frame
		odm_msg = Float64MultiArray()
		odm_msg.data.append(self.x)
		odm_msg.data.append(self.y)
		odm_msg.data.append(self.yaw)
		self.odm_pub.publish(odm_msg)

if __name__ == "__main__":
	rospy.init_node("me212bot_control", anonymous = False)
	me212bot_control = me212bot_control()
	rospy.spin()
