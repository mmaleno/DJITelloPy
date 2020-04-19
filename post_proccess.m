%% Kevin Shoyer and Max Maleno
% 4/11/2020
% This script is used for post processing from the Tello Drone Control data
%% Experiment 1: just messing around with it
 filename = '/Users/kevinshoyer/Desktop/DJITelloPy_E205/CallibrationDataLogs/Tello_Log_2020_04_11_13_31_39.csv';
%% Experiment 2: (somewhat botched)
filename = '/Users/kevinshoyer/Desktop/DJITelloPy_E205/CallibrationDataLogs/Tello_Log_2020_04_11_15_04_08.csv';
% Experiment 2: to test coordinate systems and get more intuition on velocity reading
% 
% Start on desk
% Clamp against wall and rotate, keeping against wall
% Do one full roll
% Do one full yaw
% Do one full pitch
% With drone flat, walk forward at walking speed

%% Experiment 3: testing accelerometer and roll/pitch/yaw coordinate system
filename = '/Users/kevinshoyer/Desktop/DJITelloPy_E205/CallibrationDataLogs/Tello_Log_2020_04_11_15_19_18.csv';
% sit for 10 seconds
%pitch upwards and look at cieling
% sit for 10
% roll to the right and look forwards.


%% experiment 4: roll pitch and yaw testing

%roll right 360 degrees
%pitch up 360 degrees
%yaw left 360 degrees
filename = '/Users/kevinshoyer/Desktop/DJITelloPy_E205/CallibrationDataLogs/Tello_Log_2020_04_11_15_53_46.csv';

%% experiement 5: looking at velocities

% Hold and move forward at walking speed
% Hold and move right at walking speed
% Tilt vehicle around and stay in one location
% Tilt vehicle 45 degrees forward and walk forward at walking speed
%filename = '/Users/kevinshoyer/Desktop/DJITelloPy_E205/CallibrationDataLogs/Tello_Log_2020_04_11_17_46_08.csv';

%% experiment 6: flying and velocities
% 1) fly up
% 2) fly forward (should be purely forward velocity)
% 3) fly right (purely right)
% 4) take same path back, but face towards direction of travel at all times
% (should be forward velocity, if velocity is local)
% 5) land on ground (to see what this does for height)

% this was about 9ft by 10ft path (2.7 meters by 3 meters)
filename = '/Users/kevinshoyer/Desktop/DJITelloPy_E205/CallibrationDataLogs/Tello_Log_2020_04_11_18_15_22.csv';

%% Experiment 7: moved around the room for a longer time to look at the drift while integrating velocity
filename = '/Users/kevinshoyer/Desktop/DJITelloPy_E205/CallibrationDataLogs/Tello_Log_2020_04_12_18_41_43.csv';

%% Experiment 8: moving in a square 5 times


%% Import data from text file.

delimiter = ',';

formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

fileID = fopen(filename,'r');

dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter,  'ReturnOnError', false);

% Close the text file.
fclose(fileID);

% Allocate imported array to column variable names
time = dataArray{:, 1};
pitch = dataArray{:, 2};
roll = dataArray{:, 3};
yaw = dataArray{:, 4};
v_x = dataArray{:, 5};
v_y = dataArray{:, 6};
v_z = dataArray{:, 7};
temp_low = dataArray{:, 8};
temp_high = dataArray{:, 9};
dist_tof = dataArray{:, 10};
height = dataArray{:, 11};
battery = dataArray{:, 12};
barometer = dataArray{:, 13};
flight_time = dataArray{:, 14};
a_x = dataArray{:, 15};
a_y = dataArray{:, 16};
a_z = dataArray{:, 17};

clearvars filename delimiter formatSpec fileID dataArray ans;

%% Plot the accelerations

figure(1)
title('Accelerations')
plot(time,a_x)
hold on
plot(time,a_y)
hold on
plot(time,a_z)
xlabel('Time (s)')
ylabel('acceleration')
legend('x acceleration','y acceleration','z acceleration')

%% Plot velocities

figure(2)
title('Velocities')
plot(time,v_x)
hold on
plot(time,v_y)
hold on
plot(time,v_z)
xlabel('Time (s)')
ylabel('velocity')
legend('x velocity','y velocity','z velocity')

%% Plotting the height

figure(3)
title('Height')
plot(time,height)
xlabel('time')
ylabel('height')

%% Plotting roll pitch yaw:

figure(4)
title('Roll,Pitch,Yaw')
plot(time,roll)
hold on
plot(time,pitch)
hold on
plot(time,yaw)
xlabel('Time (s)')
ylabel('angle (some units)')
legend('Roll','Pitch','Yaw')




%% numerically integrate velocity values to propegate state
delta_t = time(2:end) - time(1:end-1)



x_pos = [];
y_pos = [];
z_pos = [];
x = 0;
y = 0;
z = 0;
for i = 1:length(delta_t)
    dt = delta_t(i)
    x = x + v_x(i).*dt/10;
    y = y + v_y(i).*dt/10;
    z = y + v_z(i).*dt/10;
    x_pos = [x_pos,x];
    y_pos = [y_pos,y];
    z_pos = [z_pos,z];
    
end

figure(5)
plot(x_pos,y_pos)
title('x,y path from velocity integration')
xlabel('x position (m)')
ylabel('y Position (m)')


%% Resample because it takes forever with this many data points

N = 1 %sample one every N samples

time = time(1:N:end);
pitch = pitch(1:N:end);
roll = roll(1:N:end);
yaw = yaw(1:N:end);
a_x = a_x(1:N:end);
a_y = a_y(1:N:end);
a_z = a_z(1:N:end);


%% transform accelerations

% first transform coordinate system of accelerations
% desired local coordinate system: 
% x positive forward
% y positive right
% z positive down
% with this coordinate system:
    %Velocities match up as expected (this is the coordinate system for
    %velocity outputs
    
    %This is also roll pitch and yaw outputs
    % roll: right hand rotation about +x
    % pitch: right hand rotation about +y
    % yaw: right handed about +z

% shift data to propper local frame
a_xl = -a_x;
a_yl = -a_y;
a_zl = -a_z;
a_l = [a_xl,a_yl,a_zl]';

%roll pitch yaw in radians

roll_l = roll*pi/180;
pitch_l = pitch*pi/180;
yaw_l = yaw*pi/180;

% Find direct cosine matrix using rotation angles
a_g = [];
for i=1:length(roll_l)
    a = a_l(:,i);
    dcm_i = angle2dcm( yaw_l(i), pitch_l(i), roll_l(i));
    a_gi = dcm_i*a;
    a_g = [a_g,a_gi];
    
end

%plot the global accelerations

figure(4)
title('Global accelerations')
xlabel('Time')
ylabel('acceleration (mg)')
plot(time,a_g(1,:))
hold on
plot(time,a_g(2,:))
hold on
plot(time,a_g(3,:))
legend('x acceleration', 'y acceleration', 'z acceleration')

%% propegate the accelerations to find position

%% numerically integrate velocity values to propegate state

delta_t = time(2:end) - time(1:end-1);

x_pos_a = [];
y_pos_a = [];
z_pos_a = [];

%velocities propegated from acceleration
v_xa = [];
v_ya = [];
v_za = [];
x = 0;
y = 0;
z = 0;
v_xai = 0;
v_yai = 0;
v_zai = 0;


for i = 10:length(delta_t)
    dt = delta_t(i);
    i = i+1;
    x = x + v_xai.*dt +1/2*a_g(1,i)*dt^2;
    y = y + v_yai.*dt +1/2*a_g(2,i)*dt^2;
    z = z + v_zai.*dt +(a_g(3,i)-1000)*dt;
    
    v_xai = v_xai+a_g(1,i)*dt;
    v_yai = v_yai+a_g(2,i)*dt;
    v_zai = v_zai+(a_g(3,i)-1000)*dt;
    
    x_pos_a = [x_pos,x];
    y_pos_a = [y_pos,y];
    z_pos_a = [z_pos,z];
    
    v_xa = [v_xa,v_xai];
    v_ya = [v_ya,v_yai];
    v_za = [v_za,v_zai];
    
end

figure(6)
plot(x_pos_a,y_pos_a)


figure(7)
plot(time(11:end),v_xa)
hold on
plot(time(11:end),v_ya)
hold on
plot(time(11:end),v_za)
legend('x velocity','y velocity','z velocity')



