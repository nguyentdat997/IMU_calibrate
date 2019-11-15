#!/usr/bin/env python
# Author: khuongnv@nadrobot
import rospy
import roslib
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg   import Imu


class TwistToMotors():
	def __init__(self):
		rospy.init_node("twist_to_motors")
		nodename = rospy.get_name()
		rospy.loginfo("%s started" % nodename)

		self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32, queue_size=10)
		self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32, queue_size=10)
		self.pub_dr = rospy.Publisher('pubdr', Float32, queue_size=10)
		self.pub_offset_imu_angular = rospy.Publisher('offset_angular_pub',Float64, queue_size=10)
		self.pub_current_imu_angular = rospy.Publisher('current_angular_pub',Float64,queue_size=10)
		#-----------
		rospy.Subscriber('imu/angular',Float64,self.imu_angular_Callback)
		rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
		rospy.Subscriber('imu/accel',Vector3,self.imu_accel_Callback)
		rospy.Subscriber('imu/gyro', Vector3,self.imu_gyro_Callback)
		rospy.Subscriber('imu/qs', Quaternion,self.imu_qua_Callback)
		self.pub_imu = rospy.Publisher('imu/raw',Imu, queue_size=10)
		#-----------

		self.w = rospy.get_param("~wheel_separation", 0.34)	#distance between robot's wheels
		self.rate = rospy.get_param("~rate", 20)
		self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)

		self.left = 0
		self.right = 0
		self.time_prev_update = rospy.Time.now()
		self.dx = 0
		self.dr = 0
		self.offset_imu_angular =0
		self.current_imu_angular= 0


		vector3 = Vector3()
		qs = Quaternion()
		self.imu_raw_msg = Imu()

		self.imu_accel_x = 0
		self.imu_accel_y = 0
		self.imu_accel_z = 0
	
		self.imu_gyro_x = 0
		self.imu_gyro_y = 0
		self.imu_gyro_z = 0
	
		self.imu_qua_x = 0
		self.imu_qua_y = 0
		self.imu_qua_z = 0
		self.imu_qua_w = 0
		
		

	def spin(self):
		r = rospy.Rate(self.rate)
		time_curr_update = rospy.Time.now()
		while not rospy.is_shutdown():
			time_diff_update = (time_curr_update - self.time_prev_update).to_sec()
			if time_diff_update < self.timeout_ticks:
				self.spinOnce()
			r.sleep()
		rospy.spin()

	def spinOnce(self):
		self.right = 1.0 * self.dx + self.dr * self.w / 2.0	#caculate tangular velocity of right wheel
		self.left = 1.0 * self.dx - self.dr * self.w / 2.0	#caculate tangular velocity of left wheel	
		self.pub_lmotor.publish(self.left)
		self.pub_rmotor.publish(self.right)

		self.imu_raw_msg.linear_acceleration.x = self.imu_gyro_x;
  		self.imu_raw_msg.linear_acceleration.y = self.imu_gyro_y;
		self.imu_raw_msg.linear_acceleration.z = self.imu_gyro_z;


    		self.imu_raw_msg.angular_velocity.x = self.imu_accel_x;
    		self.imu_raw_msg.angular_velocity.y = self.imu_accel_y;
    		self.imu_raw_msg.angular_velocity.z = self.imu_accel_z;

    		
    		self.imu_raw_msg.orientation.x = self.imu_qua_x;
    		self.imu_raw_msg.orientation.y = self.imu_qua_y;
    		self.imu_raw_msg.orientation.z = self.imu_qua_z;
    		self.imu_raw_msg.orientation.w = self.imu_qua_w;


    		self.imu_raw_msg.header.stamp = rospy.Time.now();
    		self.imu_raw_msg.header.frame_id = "imu_link";
    		self.pub_imu.publish(self.imu_raw_msg);
		self.pub_dr.publish(self.dr)

	def imu_accel_Callback(self, msg):
		self.imu_accel_x = msg.x;
		self.imu_accel_y = msg.y;
		self.imu_accel_z = msg.z;
	def imu_gyro_Callback(self, msg):
		self.imu_gyro_x = msg.x;
		self.imu_gyro_y = msg.y;
		self.imu_gyro_z = msg.z;
	def imu_qua_Callback(self, msg):
		self.imu_qua_x = msg.x;
		self.imu_qua_y = msg.y;
		self.imu_qua_z = msg.z;
		self.imu_qua_w = msg.w;
	def imu_angular_Callback(self, msg):
		self.imu_angular = msg.data;
		if (self.imu_angular ==0.0):
			self.current_imu_angular = self.imu_angular
		else :
			self.offset_imu_angular = self.imu_angular
		self.pub_offset_imu_angular.publish(self.offset_imu_angular)
		self.pub_current_imu_angular.publish(self.offset_imu_angular)

	def twistCallback(self, msg):
		self.dx = msg.linear.x
		self.dr = msg.angular.z
		self.time_prev_update = rospy.Time.now()


if __name__ == '__main__':
	try:
		twistToMotors = TwistToMotors()
		twistToMotors.spin()
	except rospy.ROSInterruptException:
		pass
