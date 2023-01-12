import matplotlib.pyplot as plt
import csv
import numpy as np
import math

filepath = '/home/lenovo/catkin_ws/src/gps_driver/rosbag/gps_stationery_data/gps_stationary_data_2.csv'
#filepath = '/home/lenovo/catkin_ws/src/gps_driver/rosbag/gps_walking_data/gps_walking_data_2.csv' 

utm_e = []
utm_n = []
alt = []
time = []
lat = []
lon = []
utm_e_offset = []
utm_n_offset = []

with open(filepath,'r') as csvfile:
	data = csv.reader(csvfile, delimiter = ',')
	next(data)
	
	for rows in data:
		time.append(int(rows[1])) #the variale Header.seq is used for reference of time. The frequency of data is 1Hz
		lat.append(float(rows[4]))
		lon.append(float(rows[5]))
		alt.append(float(rows[6]))
		utm_e.append(float(rows[7]))
		utm_n.append(float(rows[8]))
		
OFFSET_E = min(utm_e)
OFFSET_N = min(utm_n)
OFFSET_TIME = min(time)

utm_e_offset = [value-OFFSET_E for value in utm_e]
utm_n_offset = [value-OFFSET_N for value in utm_n]
time_seconds = [value-OFFSET_TIME for value in time]

m, c = np.polyfit(utm_e_offset, utm_n_offset, 1)
best_fit_line_y = m*(np.array(utm_e_offset))+c
#consider 0 velocity in when time = 0
vel_e = [0] 
vel_n = [0]
vel_net = [0]

for i in range(len(time_seconds)-1):
	vel_e.append(utm_e_offset[i+1] - utm_e_offset[i])
	vel_n.append(utm_n_offset[i+1] - utm_n_offset[i])
	vel_net.append(math.sqrt((vel_e[i+1])**2 + (vel_e[i+1])**2))

error = []
for i in range(len(time)):
	error.append(abs(utm_n_offset[i] - best_fit_line_y[i]))

correlation_matrix = np.corrcoef(utm_e_offset, utm_n_offset)
correlation_xy = correlation_matrix[0,1]
r_squared = correlation_xy**2
print(r_squared)

#-------------------Plots of Stationary Data---------------------
#plt.scatter(utm_e_offset, utm_n_offset, label= "utm_e vs utm_n")
#plt.xlabel('utm_e_offset_values')
#plt.ylabel('utm_n_offset_values')
#plt.title('GPS Stationary Data (UTM_E vs UTM_N)', fontsize = 20)
#plt.legend()
#plt.show()

#plt.plot(time_seconds, alt, label='Altitude')
#plt.xlabel('time (seconds)')
#plt.ylabel('Altitude')
#plt.title('GPS Stationary Altitude Data', fontsize = 20)
#plt.legend()
#plt.show()

#---------------Plots of Walking Data-----------------------------

plt.scatter(time_seconds, vel_e, label='velocity_easting', s=2)
plt.xlabel('time(seconds)')
plt.ylabel('velocity_easting')
plt.title('GPS Walking Velocity_E Data', fontsize = 20)
plt.legend()
plt.show()

plt.scatter(time_seconds, vel_n, label='velocity_northing', s=2)
plt.xlabel('time(seconds)')
plt.ylabel('velocity_northing')
plt.title('GPS Walking Velocity_N Data', fontsize = 20)
plt.legend()
plt.show()

plt.scatter(time_seconds, vel_net, label='net_velocity', s=2)
plt.xlabel('time(seconds)')
plt.ylabel('net velocity')
plt.title('GPS Walking Net Velocity Data', fontsize = 20)
plt.legend()
plt.show()

plt.plot(time_seconds, error, label='Absolute error in UTM_N and ideal path')
plt.xlabel('time(seconds)')
plt.ylabel('error(meters)')
plt.title('GPS Walking Data Error', fontsize = 20)
plt.legend()
plt.show()

plt.scatter(utm_e_offset, utm_n_offset, s=3, label= f"utm_e vs utm_n [y={m}x + {c}]")
plt.plot(utm_e_offset, best_fit_line_y, color = 'r', label="best fit line")
plt.xlabel('utm_e_offset_values')
plt.ylabel('utm_n_offset_values')
plt.title('GPS Walking Data (UTM_E vs UTM_N)', fontsize = 20)
plt.legend()
plt.show()

plt.scatter(time_seconds, alt, label='Altitude', s=3)
plt.xlabel('Time')
plt.ylabel('Altitude')
plt.title('GPS Walking Altitude Data', fontsize = 20)
plt.legend()
plt.show()

plt.plot(time_seconds, utm_e_offset, label='UTM Easting')
plt.xlabel('Time(seconds)')
plt.ylabel('utm_e_offset_values')
plt.title('GPS Walking UTM_E Data', fontsize = 20)
plt.legend()
plt.show()

plt.plot(time_seconds, utm_n_offset, label="UTM Northing")
plt.xlabel('Time(seconds)')
plt.ylabel('utm_n_offset_values')
plt.title('GPS Walking UTM_N Data', fontsize = 20)
plt.legend()
plt.show()
