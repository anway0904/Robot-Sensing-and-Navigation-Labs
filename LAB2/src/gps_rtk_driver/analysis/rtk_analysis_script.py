import matplotlib.pyplot as plt
import csv
import numpy as np
import math
import os
import pathlib
import statistics as stat

filepath = '/home/lenovo/catkin_ws/src/gps_rtk_driver/rosbag'

utm_e	= {'stat':[], 'walk':[], 'stat_occ':[], 'walk_occ':[]}
utm_n	= {'stat':[], 'walk':[], 'stat_occ':[], 'walk_occ':[]}
alt		= {'stat':[], 'walk':[], 'stat_occ':[], 'walk_occ':[]}
utm_e_offset = {'stat':[], 'walk':[], 'stat_occ':[], 'walk_occ':[]}
utm_n_offset = {'stat':[], 'walk':[], 'stat_occ':[], 'walk_occ':[]}

def get_bestfit_line(point_1, point_2, data_type):
	utm_e_data = utm_e_offset[data_type][list(np.around(utm_e_offset[data_type], 3)).index(point_1[0]) : list(np.around(utm_e_offset[data_type], 3)).index(point_2[0])]
	utm_n_data = utm_n_offset[data_type][list(np.around(utm_n_offset[data_type], 3)).index(point_1[1]) : list(np.around(utm_n_offset[data_type], 3)).index(point_2[1])]
	m, c = np.polyfit(utm_e_data, utm_n_data, 1)
	best_fit_line_y = m*(np.array(utm_e_data)) + c
	return(utm_e_data, utm_n_data, best_fit_line_y, m, c)
	
def append_data_to_dict(filename, key):
	with open(filename, 'r') as csvfile:
		data = csv.reader(csvfile, delimiter = ',')
		next(data)
		
		for rows in data:
			alt[key].append(float(rows[6]))
			utm_e[key].append(float(rows[7]))
			utm_n[key].append(float(rows[8]))
			
		OFFSET_E = utm_e[key][0]
		OFFSET_N = utm_n[key][0]

		utm_e_offset[key] = [value-OFFSET_E for value in utm_e[key]]
		utm_n_offset[key] = [value-OFFSET_N for value in utm_n[key]]

for folder in os.listdir(filepath):
	subfolder = os.path.join(filepath, folder)
	for filename in os.listdir(subfolder):
		if(pathlib.Path(filename).suffix == '.csv'):
			if 'stat' in filename:
				if 'occluded' not in filename:
					append_data_to_dict(os.path.join(subfolder, filename), 'stat')
					
				else:
					append_data_to_dict(os.path.join(subfolder, filename), 'stat_occ')
					
			else:
				if 'occluded' not in filename:
					append_data_to_dict(os.path.join(subfolder, filename), 'walk')
					
				else:
					append_data_to_dict(os.path.join(subfolder, filename), 'walk_occ')

#-------------------Plots of Clear Data----------------------------------------------
unique_points = {}
for i,j in zip(utm_e_offset['stat'], utm_n_offset['stat']):
	if (i,j) not in unique_points.keys():
		unique_points[(i, j)] = 1
	else:
		unique_points[(i,j)] += 1

print(unique_points)
plt.scatter(utm_e_offset['stat'], utm_n_offset['stat'], label= "UTM_E vs UTM_N")
plt.xlabel('utm_e_offset_values (meters)')
plt.ylabel('utm_n_offset_values (meters)')
plt.title('GPS RTK Stationary Data (No occlusion)', fontsize = 15)
for i,j in zip(np.unique(np.array(utm_e_offset['stat'])), np.unique(np.array(utm_n_offset['stat']))):
	plt.text(i, j, '({}, {})'.format(round(i,5), round(j, 5)))
plt.legend()


plt.show()
RMSE = np.sqrt((max(utm_e_offset['stat']) - min(utm_e_offset['stat']))**2 + (max(utm_n_offset['stat']) - min(utm_n_offset['stat']))**2)
print('RMSE Stationary No-Occlusion: ', RMSE)


corners_walk = [(0, 0), (9.683, -9.626), (-1.078, -20.436), (-10.507, -11.075), (0.108, -0.095)]
error = []
utm_e_data, utm_n_data, best_fit_line_y, m, c = get_bestfit_line(corners_walk[0], corners_walk[1], 'walk')
plt.plot(utm_e_data, best_fit_line_y, color='red')
for x1, y1 in zip(utm_e_data, utm_n_data):
	error.append(abs((m * x1 - y1 + c)) / (math.sqrt(m*m + 1)))
RMSE_1 = np.sqrt(np.square(np.array(error)).mean())

error = []
utm_e_data, utm_n_data, best_fit_line_y, m, c = get_bestfit_line(corners_walk[1], corners_walk[2], 'walk')
plt.plot(utm_e_data, best_fit_line_y, color='red')
for x1, y1 in zip(utm_e_data, utm_n_data):
	error.append(abs((m * x1 - y1 + c)) / (math.sqrt(m*m + 1)))
RMSE_2 = np.sqrt(np.square(np.array(error)).mean())

error = []
utm_e_data, utm_n_data, best_fit_line_y, m, c = get_bestfit_line(corners_walk[2], corners_walk[3], 'walk')
plt.plot(utm_e_data, best_fit_line_y, color='red')
for x1, y1 in zip(utm_e_data, utm_n_data):
	error.append(abs((m * x1 - y1 + c)) / (math.sqrt(m*m + 1)))
RMSE_3 = np.sqrt(np.square(np.array(error)).mean())

error = []
utm_e_data, utm_n_data, best_fit_line_y, m, c = get_bestfit_line(corners_walk[3], corners_walk[4], 'walk')
plt.plot(utm_e_data, best_fit_line_y, color='red', label='Best fit lines')
for x1, y1 in zip(utm_e_data, utm_n_data):
	error.append(abs((m * x1 - y1 + c)) / (math.sqrt(m*m + 1)))
RMSE_4 = np.sqrt(np.square(np.array(error)).mean())

print('RMSE Walking Non-Occluded: ', (RMSE_1+RMSE_2+RMSE_3+RMSE_4)/4)

plt.scatter(utm_e_offset['walk'], utm_n_offset['walk'], label= "UTM_E vs UTM_N")
plt.scatter([x[0] for x in corners_walk if x!=(0.108, -0.095)], [y[1] for y in corners_walk if y!=(0.108, -0.095)], label='Corner points')
plt.xlabel('utm_e_offset_values (meters)')
plt.ylabel('utm_n_offset_values (meters)')
plt.title('GPS RTK Walking Data (No occlusion)', fontsize = 15)

for point in corners_walk:
	if point == (0.108, -0.095):
			break
	plt.text(point[0]+0.7, point[1], f'({round(point[0], 3)}, {round(point[1], 3)})')
	
plt.legend()
plt.show()

#-------------------Plots of Occluded Data---------------------------------------------
median_utm_e = stat.median(utm_e_offset['stat_occ'])
median_utm_n = stat.median(utm_n_offset['stat_occ'])

unique_points = {}
for i,j in zip(utm_e_offset['stat_occ'], utm_n_offset['stat_occ']):
	i = round(i, 3)
	j = round(j, 3)
	if (i,j) not in unique_points.keys():
		unique_points[(i, j)] = 1
	else:
		unique_points[(i,j)] += 1

plt.bar([str(key) for key in unique_points.keys()], unique_points.values(), width=0.5, color='#FFA500', edgecolor='white', label='Frequency of data')
for x, y in enumerate(unique_points.values()):
	plt.text(x, y+1, str(y), horizontalalignment='center', fontweight='bold')
plt.title('Frequency of Overlapping Points in Stationary Data (Occluded)', fontsize = 15)
plt.xticks(rotation=30, ha='right')
plt.xlabel('data points')
plt.ylabel('number of overlapping points')
plt.grid(axis = 'y')
plt.legend()
plt.show()

for r in range(0, 80):
	r = r/1000
	count = 0
	for point in unique_points:
		if ((point[0] - median_utm_e)**2 + (point[1] - median_utm_n)**2 <= r**2):
			count += unique_points[point]
	if count >= 0.95*(len(utm_e_offset['stat_occ'])):
		break
		
figure, axes = plt.subplots() 
circle = plt.Circle((median_utm_e, median_utm_n), r, fill=False, color='red', linestyle='--', label='96.8% data enclosed') 
axes.add_patch(circle)

plt.scatter(utm_e_offset['stat_occ'], utm_n_offset['stat_occ'], label= "UTM_E vs UTM_N")
plt.scatter(median_utm_e, median_utm_n)
plt.xlabel('utm_e_offset_values (meters)')
plt.ylabel('utm_n_offset_values (meters)')
plt.title('GPS RTK Stationary Data (Occluded)', fontsize = 15)
for point in unique_points:
	plt.text(point[0]+0.001, point[1], f"({point[0]}, {point[1]})")
plt.legend()
plt.show()
RMSE = np.sqrt((max(utm_e_offset['stat_occ']) - min(utm_e_offset['stat_occ']))**2 + (max(utm_n_offset['stat_occ']) - min(utm_n_offset['stat_occ']))**2)
print('RMSE Stationary Occlusion: ', RMSE)


corners_walk_occ = [(0, 0), (-14.384, -7.239), (-18.045, 0.314), (-3.882, 7.503), (-0.025, 0.093)]
error = []
utm_e_data, utm_n_data, best_fit_line_y, m, c = get_bestfit_line(corners_walk_occ[0], corners_walk_occ[1], 'walk_occ')
plt.plot(utm_e_data, best_fit_line_y, color='red', alpha=0.7)
for x1, y1 in zip(utm_e_data, utm_n_data):
	error.append(abs((m * x1 - y1 + c)) / (math.sqrt(m*m + 1)))
RMSE_1 = np.sqrt(np.square(np.array(error)).mean())

error = []
utm_e_data, utm_n_data, best_fit_line_y, m, c = get_bestfit_line(corners_walk_occ[1], corners_walk_occ[2], 'walk_occ')
plt.plot(utm_e_data, best_fit_line_y, color='red')
for x1, y1 in zip(utm_e_data, utm_n_data):
	error.append(abs((m * x1 - y1 + c)) / (math.sqrt(m*m + 1)))
RMSE_2 = np.sqrt(np.square(np.array(error)).mean())

error = []
utm_e_data, utm_n_data, best_fit_line_y, m, c = get_bestfit_line(corners_walk_occ[2], corners_walk_occ[3], 'walk_occ')
plt.plot(utm_e_data, best_fit_line_y, color='red')
for x1, y1 in zip(utm_e_data, utm_n_data):
	error.append(abs((m * x1 - y1 + c)) / (math.sqrt(m*m + 1)))
RMSE_3 = np.sqrt(np.square(np.array(error)).mean())

error = []
utm_e_data, utm_n_data, best_fit_line_y, m, c = get_bestfit_line(corners_walk_occ[3], corners_walk_occ[4], 'walk_occ')
plt.plot(utm_e_data, best_fit_line_y, color='red', label='Best fit lines')
for x1, y1 in zip(utm_e_data, utm_n_data):
	error.append(abs((m * x1 - y1 + c)) / (math.sqrt(m*m + 1)))
RMSE_4 = np.sqrt(np.square(np.array(error)).mean())

print('RMS Error Walking Occluded: ', (RMSE_1 + RMSE_2 + RMSE_3 + RMSE_4)/4)

plt.scatter(utm_e_offset['walk_occ'], utm_n_offset['walk_occ'], label= "UTM_E vs UTM_N")
plt.scatter([x[0] for x in corners_walk_occ if x!=(-0.025, 0.093)], [y[1] for y in corners_walk_occ if y!=(-0.025, 0.093)], label='Corner points')
plt.xlabel('utm_e_offset_values (meters)')
plt.ylabel('utm_n_offset_values (meters)')
plt.title('GPS RTK walking Data (Occluded)', fontsize = 15)
for point in corners_walk_occ:
	if point == (-0.025, 0.093):
		break
	plt.text(point[0]+0.7, point[1]-0.3, f'({round(point[0], 3)}, {round(point[1], 3)})')
plt.legend()
plt.show()
