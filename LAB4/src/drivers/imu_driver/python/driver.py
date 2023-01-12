#!/usr/bin/env python3

from serial import Serial
from serial.serialutil import SerialException
from imu_driver.msg import imu_msg
import rospy
import sys 
import numpy as np
import math
from sensor_msgs.msg import Imu

BAUD_RATE = 115200
TIMEOUT = 1

def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):

	#converting yaw, pitch, roll from degree to radians
	roll, pitch, yaw = roll_deg*(math.pi/180), pitch_deg*(math.pi/180), yaw_deg*(math.pi/180) 
	
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	
	return ([qx, qy, qz, qw])

if __name__ == '__main__':
	rospy.init_node('imu_driver')
	serial_port = sys.argv[1]
	
	try:
		ser = Serial(serial_port, BAUD_RATE, timeout=TIMEOUT)
	except SerialException:
		rospy.logerr('Could not connect to the serial port {}'.format(serial_port))
	
	#To set the IMU rate at 40 Hz
	ser.write(b"$VNWRG, 07, 40*XX")
	
	imu_pub = rospy.Publisher('imu', imu_msg, queue_size=10)
	
	pub_msg = Imu()
	imu_message = imu_msg()
	
	seq_counter = 1
	
	while not rospy.is_shutdown():
		try:
			try:
				line = str(ser.readline().strip().decode("UTF-8"))
				
			except UnicodeDecodeError:
				rospy.logerr("UnicodeDecodeError Occured. Ignoring..")
				line = ' '
				pass
			
			if "VNYMR" in line:
				
				imu_message.VNYMR = line
				vnymr_data = line.split(',')
				
				yaw_deg, pitch_deg, roll_deg = float(vnymr_data[1]), float(vnymr_data[2]), float(vnymr_data[3])
				mag_x, mag_y, mag_z = float(vnymr_data[4]), float(vnymr_data[5]), float(vnymr_data[6])
				acc_x, acc_y, acc_z = float(vnymr_data[7]), float(vnymr_data[8]), float(vnymr_data[9])
				ang_x, ang_y, ang_z = float(vnymr_data[10]), float(vnymr_data[11]), float(vnymr_data[12][:-3])
				
				quaternion_list = euler_to_quaternion(roll_deg, pitch_deg, yaw_deg)
				time_now = rospy.get_rostime()
				
				imu_message.Header.frame_id = 'IMU1_Frame'
				imu_message.Header.stamp = time_now
				imu_message.Header.seq = seq_counter
				
				imu_message.IMU.header.frame_id = 'IMU1_Frame'
				imu_message.IMU.header.stamp = time_now
				imu_message.IMU.header.seq = seq_counter
				
				imu_message.IMU.orientation.x = quaternion_list[0]
				imu_message.IMU.orientation.y = quaternion_list[1]
				imu_message.IMU.orientation.z = quaternion_list[2]
				imu_message.IMU.orientation.w = quaternion_list[3]
				
				imu_message.IMU.angular_velocity.x = ang_x
				imu_message.IMU.angular_velocity.y = ang_y
				imu_message.IMU.angular_velocity.z = ang_z
				
				imu_message.IMU.linear_acceleration.x = acc_x
				imu_message.IMU.linear_acceleration.y = acc_y
				imu_message.IMU.linear_acceleration.z = acc_z
				
				imu_message.MagField.header.frame_id = 'IMU1_Frame'
				imu_message.MagField.header.stamp = time_now
				imu_message.MagField.header.seq = seq_counter
				
				imu_message.MagField.magnetic_field.x = mag_x
				imu_message.MagField.magnetic_field.y = mag_y
				imu_message.MagField.magnetic_field.z = mag_z
				
				imu_pub.publish(imu_message)
				seq_counter += 1
				
				print('\n')
				print(f'yaw_deg = {yaw_deg}\tpitch_deg = {pitch_deg}\troll_deg = {roll_deg}')
#				print(f'mag_x = {mag_x}\tmag_y = {mag_y}\tmag_z = {mag_z}')
#				print(f'acc_x = {acc_x}\t\tacc_y = {acc_y}\t\tacc_z = {acc_z}')
#				print(f'ang_x = {ang_x}\tang_y = {ang_y}\tang_z = {ang_z}')
#				
		except rospy.ROSInterruptException:
			ser.close()
