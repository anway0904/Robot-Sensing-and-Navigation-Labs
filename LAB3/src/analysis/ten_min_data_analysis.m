data = readtable('5_min_data_2.csv');
time = data.Time - min(data.Time);
%-------------------------------------------------------------------
%Accelerometer 
acc_x = data.IMU_linear_acceleration_x;
acc_y = data.IMU_linear_acceleration_y;
acc_z = data.IMU_linear_acceleration_z;
%-------------------------------------------------------------------
%Accelerometer Plots
%-------------------------------------------------------------------
figure();
subplot(1, 2, 1);
plot(time, acc_x);
title('Accelerometer X', FontSize=18);
xlabel('time (seconds)', FontSize=16);
ylabel('acceleration (m/s^2)', FontSize=16);
grid minor;

subplot(1, 2, 2);
histogram(acc_x);
fprintf('Acc X: Mean = %f, SD = %f\n', mean(acc_x), std(acc_x));
title('Histogram of Accelerometer X', FontSize=18);
xlabel('acceleration (m/s^2)', FontSize=16);
ylabel('frequency', FontSize=16);
grid minor;
%-------------------------------------------------------------------
figure();
subplot(1, 2, 1);
plot(time, acc_y);
title('Accelerometer Y', FontSize=18);
xlabel('time (seconds)', FontSize=16);
ylabel('acceleration (m/s^2)', FontSize=16);
grid minor;

subplot(1, 2, 2);
histogram(acc_y, BinWidth=0.0025);
fprintf('Acc Y: Mean = %f, SD = %f\n', mean(acc_y), std(acc_y));
title('Histogram of Accelerometer Y', FontSize=18);
xlabel('acceleration (m/s^2)', FontSize=16);
ylabel('frequency', FontSize=16);
grid minor;
%-------------------------------------------------------------------
figure();
subplot(1, 2, 1);
plot(time, acc_z);
title('Accelerometer Z', FontSize=18);
xlabel('time (seconds)', FontSize=16);
ylabel('acceleration (m/s^2)', FontSize=16);
grid minor;

subplot(1, 2, 2);
histogram(acc_z);
fprintf('Acc Z: Mean = %f, SD = %f\n', mean(acc_z), std(acc_z));
title('Histogram of Accelerometer Z', FontSize=18);
xlabel('acceleration (m/s^2)', FontSize=16);
ylabel('frequency', FontSize=16);
grid minor;
%-------------------------------------------------------------------
%Gyroscope
omega_x = data.IMU_angular_velocity_x;
omega_y = data.IMU_angular_velocity_y;
omega_z = data.IMU_angular_velocity_z;
%-------------------------------------------------------------------
%Gyroscope Plots
%-------------------------------------------------------------------
figure();
subplot(1, 2, 1);
plot(time, omega_x);
title('Gyroscope X', FontSize=18);
xlabel('time (seconds)', FontSize=16);
ylabel('angular velocity (rad/s^2)', FontSize=16);
grid minor;

subplot(1, 2, 2);
histogram(omega_x);
fprintf('Omega X: Mean = %f, SD = %f\n', mean(omega_x), std(omega_x));
title('Histogram of Gyroscope X', FontSize=18);
xlabel('angular velocity (rad/s^2)', FontSize=16);
ylabel('frequency', FontSize=16);
grid minor;
%-------------------------------------------------------------------
figure();
subplot(1, 2, 1);
plot(time, omega_y);
title('Gyroscope Y', FontSize=18);
xlabel('time (seconds)', FontSize=16);
ylabel('angular velocity (rad/s^2)', FontSize=16);
grid minor;

subplot(1, 2, 2);
histogram(omega_y, BinWidth=0.0002);
fprintf('Omega Y: Mean = %f, SD = %f\n', mean(omega_y), std(omega_y));
title('Histogram of Gyroscope Y', FontSize=18);
xlabel('angular velocity (rad/s^2)', FontSize=16);
ylabel('frequency', FontSize=16);
grid minor;
%-------------------------------------------------------------------
figure();
subplot(1, 2, 1);
plot(time, omega_z);
title('Gyroscope Z', FontSize=18);
xlabel('time (seconds)', FontSize=16);
ylabel('angular velocity (rad/s^2)', FontSize=16);
grid minor;

subplot(1, 2, 2);
histogram(omega_z);
fprintf('Omega Z: Mean = %f, SD = %f\n', mean(omega_z), std(omega_z));
title('Histogram of Gyroscope Z', FontSize=18);
xlabel('angular velocity (rad/s^2)', FontSize=16);
ylabel('frequency', FontSize=16);
grid minor;
%-------------------------------------------------------------------
%IMU Magnetometer
mag_x = data.MagField_magnetic_field_x;
mag_y = data.MagField_magnetic_field_y;
mag_z = data.MagField_magnetic_field_z;
%-------------------------------------------------------------------
%Magnetometer Plots
%-------------------------------------------------------------------
figure();
subplot(1, 2, 1);
plot(time, mag_x);
title('Magnetometer X', FontSize=18);
xlabel('time (seconds)', FontSize=16);
ylabel('magnetic field (gauss)', FontSize=16);
grid minor;

subplot(1, 2, 2);
histogram(mag_x, BinWidth=0.0009);
fprintf('Mag X: Mean = %f, SD = %f\n', mean(mag_x), std(mag_x));
title('Histogram of Magnetometer X', FontSize=18);
xlabel('magnetic field (gauss)', FontSize=16);
ylabel('frequency', FontSize=16);
grid minor;
%-------------------------------------------------------------------
figure();
subplot(1, 2, 1);
plot(time, mag_y);
title('Magnetometer Y', FontSize=18);
xlabel('time (seconds)', FontSize=16);
ylabel('magnetic field (gauss)', FontSize=16);
grid minor;

subplot(1, 2, 2);
histogram(mag_y, BinWidth=0.001);
fprintf('Mag Y: Mean = %f, SD = %f\n', mean(mag_y), std(mag_y));
title('Histogram of Magnetometer Y', FontSize=18);
xlabel('magnetic field (gauss)', FontSize=16);
ylabel('frequency', FontSize=16);
grid minor;
%-------------------------------------------------------------------
figure();
subplot(1, 2, 1);
plot(time, mag_z);
title('Magnetometer Z', FontSize=18);
xlabel('time (seconds)', FontSize=16);
ylabel('magnetic field (gauss)', FontSize=16);
grid minor;

subplot(1, 2, 2);
histogram(mag_z, BinWidth=0.002);
fprintf('Mag Z: Mean = %f, SD = %f\n', mean(mag_z), std(mag_z));
title('Histogram of Magnetometer Z', FontSize=18);
xlabel('magnetic field (gauss)', FontSize=16);
ylabel('frequency', FontSize=16);
grid minor;
%-------------------------------------------------------------------
%Orientation
qx = data.IMU_orientation_x;
qy = data.IMU_orientation_y;
qz = data.IMU_orientation_z;
qw = data.IMU_orientation_w;

t0 = 2.0 * (qw .* qx + qy .* qz);
t1 = 1.0 - 2.0 .* (qx .* qx + qy .* qy);
roll = atan(t0 ./ t1);

t2 = 2.0 .* (qw .* qy - qz .* qx);
if (t2 > 1.0)
    t2 = 1.0;
end

if (t2 < -1.0)
    t2 = -1;
end

pitch = asin(t2);

t3 = 2.0 .* (qw .* qz + qx .* qy);
t4 = 1.0 - 2.0 .* (qy .* qy + qz .* qz);
yaw = atan(t3 ./ t4);

yaw = yaw*180/pi;
pitch = pitch*180/pi;
roll = roll*180/pi;

%-------------------------------------------------------------------
%Orientation Plots
%-------------------------------------------------------------------
figure();
subplot(1, 2, 1);
plot(time, yaw);
title('Yaw', FontSize=18);
xlabel('time (seconds)', FontSize=16);
ylabel('yaw (degrees)', FontSize=16);
grid minor;

subplot(1, 2, 2);
histogram(yaw, BinWidth=0.02);
fprintf('Yaw: Mean = %f, SD = %f\n', mean(yaw), std(yaw))
title('Histogram of Yaw', FontSize=18);
xlabel('yaw (degrees)', FontSize=16);
ylabel('frequency', FontSize=16);
grid minor;
%-------------------------------------------------------------------
figure();
subplot(1, 2, 1);
plot(time, pitch);
title('Pitch', FontSize=18);
xlabel('time (seconds)', FontSize=16);
ylabel('pitch (degrees)', FontSize=16);
grid minor;

subplot(1, 2, 2);
histogram(pitch, BinWidth=0.002);
fprintf('Pitch: Mean = %f, SD = %f\n', mean(pitch), std(pitch))
title('Histogram of Pitch', FontSize=18);
xlabel('pitch (degrees)', FontSize=16);
ylabel('frequency', FontSize=16);
grid minor;
%-------------------------------------------------------------------
figure();
subplot(1, 2, 1);
plot(time, roll);
title('Roll', FontSize=18);
xlabel('time (seconds)', FontSize=16);
ylabel('roll (degrees)', FontSize=16);
grid minor;

subplot(1, 2, 2);
histogram(roll, BinWidth = 0.002, FaceColor="#0000FF", FaceAlpha=0.5);
fprintf('Roll: Mean = %f, SD = %f\n', mean(roll), std(roll))
title('Histogram of Roll', FontSize=18);
xlabel('roll (degrees)', FontSize=16);
ylabel('frequency', FontSize=16);
hold on;
grid minor;