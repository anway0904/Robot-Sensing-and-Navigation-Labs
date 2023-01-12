#!/usr/bin/env python3

from serial import Serial
from serial.serialutil import SerialException
from gps_driver.msg import gps_msg
import rospy
import utm
import sys 
from std_msgs.msg import String


BAUD_RATE = 4800
TIMEOUT = 1

def get_lat_long(gpgga_list):
	lat_decimal = float(gpgga_list[2][:2]) + float(gpgga_list[2][2:])/60
	#print(float(gpgga_list[2][2:]), gpgga_list[2])
	lon_decimal = float(gpgga_list[4][:3]) + float(gpgga_list[4][3:])/60
	#print(lat_decimal, lon_decimal)
	lat_dir = gpgga_list[3]
	lon_dir = gpgga_list[5]
	
	latitude = lat_decimal if  lat_dir!='S' else (-1)*lat_decimal
	longitude = lon_decimal if lon_dir!='W' else (-1)*lon_decimal
	
	return(latitude, longitude)

def get_lat_long_utm(lat, lon):
	return(utm.from_latlon(lat, lon))

def get_seconds():
	hours, minutes, seconds = float(gpgga_list[1][:2]), float(gpgga_list[1][2:4]), float(gpgga_list[1][4:])
	total_secs = hours*3600 + minutes*60 + seconds
	return(total_secs)
	
if __name__ == '__main__':
	rospy.init_node('gps_driver')
	serial_port = sys.argv[1]
	ser = Serial(serial_port, BAUD_RATE, timeout=TIMEOUT)
	
	gps_pub = rospy.Publisher('gps', gps_msg, queue_size=10)
	gpgga_pub = rospy.Publisher('gpgga', String, queue_size=10)
	
	gps_message = gps_msg()
	gpgga_message = String()
	
	seq_counter = 1
	
	while not rospy.is_shutdown():
		try:
			line = str(ser.readline().strip().decode("UTF-8"))
			if line.startswith("$GPGGA"):
				gpgga_message.data = line
				gpgga_list = line.split(',')
				utc_time = gpgga_list[1][:2] + gpgga_list[1][2:4] + gpgga_list[1][4:]
				
				latitude, longitude = get_lat_long(gpgga_list)
				altitude = float(gpgga_list[9])
				utm_easting, utm_northing, zone_number, zone_letter = get_lat_long_utm(latitude, longitude)
				
				gps_message.Latitude = latitude
				gps_message.Longitude = longitude
				gps_message.Altitude = altitude
				gps_message.UTM_easting = utm_easting
				gps_message.UTM_northing = utm_northing
				gps_message.Zone = zone_number
				gps_message.Letter = zone_letter
				gps_message.Header.seq = seq_counter
				gps_message.Header.stamp = rospy.Time.from_sec(get_seconds())
				gps_message.Header.frame_id = "GPS1_Frame"
				
				gps_pub.publish(gps_message)
				gpgga_pub.publish(gpgga_message)
				seq_counter += 1
				
				print(gpgga_list[2], gpgga_list[4])
				print(f"\n\nUTC Time: {utc_time}\nLatitude: {latitude}\nLongitude: {longitude}\nAltitude: {altitude}\nUTM Easting: {utm_easting}\nUTM Northing: {utm_northing}\nZone Number: {zone_number}\nZone Letter: {zone_letter}")
				
		except rospy.ROSInterruptException:
			ser.close()
