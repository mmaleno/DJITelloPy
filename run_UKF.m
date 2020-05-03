%% Unscented Kalman Filter
% Max Maleno and Kevin Shoyer
% 4/23/2020

%Functionallity: This Script runs an Unscented Kalman Filter to fuse IMU
%and computer vision data to estimate a quadcopter's position

clear all
clear figure

% To Dos: 
% 
% 1. Create plots for final deliverable
% 2. Make Case statements
%     - Different motion models
%     - Update velocity with tag measurements to recallibrate
%     - Different R_t approaches
%     - Change correction frequency
% 3. Different path
%     - rectangular
%     - complex
%     - faster velocity


global a b k lambda nmu wm wc tag_coords corrfreq
global method

%Choose the method used for this trial
method = 1;
corrfreq = 1;%only effects method 1
%methods:
 %0: run without updating velocity
 %1: run with updating velocity and update velocity and only see a tag every 1/corrfreq of the time

a = 0.75; %alpha, controls spread of sigma points
b = 2; %beta, 2 is best for gaussian state distributions
k = 0; %commonly set to zero
nmu = 9; %number of states in state vector
lambda = a^2*(nmu+k)-nmu;

wm = [lambda/(nmu+lambda);1/(2*(nmu+lambda))*ones(2*nmu,1)];
wc = [lambda/(nmu+lambda)+(1-a^2+b);1/(2*(nmu+lambda))*ones(2*nmu,1)];

% Coordinates of tag(s) in global frame
% Tag order: Tag36h11
%tag_coords = [2.74,0.2286,-0.6731]';%for linear test
tag_coords = [2.74,34*.0254,-0.6731]';%for rectangular test

% Load Data

% Kev comp
% filename = '/Users/kevinshoyer/Desktop/DJITelloPy_E205.nosync/clean_trials/Linear_imu.csv';
% filename_april = '/Users/kevinshoyer/Desktop/DJITelloPy_E205.nosync/clean_trials/Linear_april.csv';

filename = '/Users/kevinshoyer/Desktop/DJITelloPy_E205.nosync/clean_trials/Rectangle_imu.csv';
filename_april = '/Users/kevinshoyer/Desktop/DJITelloPy_E205.nosync/clean_trials/Rectangle_april.csv';


% Max comp
% filename = 'clean_trials/Linear_imu.csv';
% filename_april = 'clean_trials/Linear_april.csv';


delimiter = ',';
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter,  'ReturnOnError', false);
fclose(fileID);

% Allocate imported array to column variable names and convert to propper
% SI units and local coordinate system. 

%coordinate system:
    % x positive forward
    % y positive right
    % z positive down
    % roll, pitch, yaw follow right hand rotations about x,y,z respectively
    
time = dataArray{:, 1};%seconds
pitch = dataArray{:, 2}.*pi/180;%radians
roll = dataArray{:, 3}.*pi/180;%radians
yaw = dataArray{:, 4}.*pi/180;%radians
v_x = dataArray{:, 5}./10;%m/s
v_y = dataArray{:, 6}./10;%m/s
v_z = dataArray{:, 7}./10;%m/s
temp_low = dataArray{:, 8};
temp_high = dataArray{:, 9};
dist_tof = dataArray{:, 10};
height = -dataArray{:, 11}./100;%m
battery = dataArray{:, 12};
barometer = dataArray{:, 13};
flight_time = dataArray{:, 14};
a_x = dataArray{:, 15}.*(-1/1000*9.81);%m/s^2
a_y = dataArray{:, 16}.*(-1/1000*9.81);%m/s^2
a_z = dataArray{:, 17}.*(-1/1000*9.81);%m/s^2

clearvars filename delimiter formatSpec fileID dataArray ans;

% import apriltag measurment data
delimiter = ',';
formatSpec = '%f%f%f%f%f%f%[^\n\r]';
fileID = fopen(filename_april,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN, 'ReturnOnError', false);
fclose(fileID);
% Allocate imported array to column variable names
frameNum = dataArray{:, 1};
timeCam = dataArray{:, 2};
tagDetected = dataArray{:, 3};
y_tag = dataArray{:, 4}; % in camera coordinate system
z_tag = dataArray{:, 5};
x_tag = dataArray{:, 6};
%if tag callibration desired
% y_tag = dataArray{:, 4}*0.6096/.5877; % in our local coordinate system
% z_tag = dataArray{:, 5}*0.6096/.5877;
% x_tag = dataArray{:, 6}*0.6096/.5877;
clearvars filename_april delimiter formatSpec fileID dataArray ans;

timeCamA = timeCam+(time(1)-timeCam(1));

%% Callibrate accelerometers using stationary data. This gets rid of offsets due to misallignment of accelerometer
% for linear test 
% % callibration procedure will occur when the vehicle is on a flat surface
% [start_data,time_calstart] = find(time>36.5);
% [takeoff,time_calend] = find(time>38);
% 
% % [indices_real_data,accels] = find(a_z ~= -1); % finds indices and values for hieghts not equal to zero
% % [takeoff_point,accels] = find(a_z > 10.5); % finds indices and values for hieghts not equal to zero
% % start_data= indices_real_data(1);
% % takeoff = takeoff_point(1); % index for when the drone takes off. Callibration procedures will end
% 
% % calculate offsets during callibration procedure
% ax_bias = mean(a_x(start_data(1):takeoff(1)));
% ay_bias = mean(a_y(start_data(1):takeoff(1)));
% az_bias = mean(a_z(start_data(1):takeoff(1)))-9.81; %assuming z is alligned with gravity
% 
% % delete these offsets from data
% a_x = a_x - ax_bias;
% a_y = a_y - ay_bias;
% a_z = a_z - az_bias;

% for rectangular test
takeoff = 3;


%% Initialize the filter

% Empty vectors to fill
nt = length(time);

SIGMA_POINTS = zeros(nmu,2*nmu+1,nt);
STATE_ESTIMATES = zeros(nmu,nt);
SIGMA = zeros(nmu,nmu,nt);

%initial starting point

mu_prev = [0;0;0;0;0;0;0;0;0];

Sigma_prev = diag([.01,.01,.01,.001,.001,.001,.01,.01,.01]); % None of these can be zero! (

sigma_points_prev = get_sigma_points(mu_prev,Sigma_prev);


%% Main Filter
%%%%%%%%%%%%%% Main Kalman Filter Algorithm %%%%%%%%%%%%%%%

% Filter Setup
% 1 	IMU calibration
% 2	Initialize values
% 	2.1	Set starting sigma points
% 	2.2	Set constants (alpha, n, kappa)
% Iterate through time steps
% 3	Collect data
% 3.1	Get IMU data
% 3.2	Get camera image
% 3.3	Calculate Apriltag pose
% 4	Prediction Step (if IMU data received)
% 	4.1	Propagate sigma points thru motion model
% 	4.2	Calculate mean prediction
% 	4.3	Calculate mean covariance prediction
% 5 	Correction Step (if AprilTag detected)
% 	5.1	Calculate predicted measurement points
% 	5.2  	Calculate mean predicted measurement
% 	5.3	Calculate measurement model uncertainty
% 	5.5	Calculate cross-covariance
% 	5.6	Calculate Kalman Gain
% 	5.7	Calculate state estimate & covariance
v_var_prev = 0; %previous variance in velocity
index_april = 1; % index to track video frame
apriltag_detections = 0;
lastdetection = 0;
lastcoordinates = [];

for t = (takeoff-1):length(time)
%for t = 114:length(time)
    %% Set variables for loop
    dt = time(t)-time(t-1);
    U_t = [a_x(t), a_y(t), a_z(t), roll(t), pitch(t), yaw(t)]';
    
    %% Prediction Step
    %Calculate the predicted state by propegating through the motion model
    
    sigma_points_prop = zeros(nmu,2*nmu+1); %will fill
    mu_bar = zeros(nmu,1); %will fill
    
    for i = 1:2*nmu+1
        sigma_points_prop(:,i) = propagate_state(sigma_points_prev(:,i),U_t,dt);
        mu_bar = mu_bar + wm(i)*sigma_points_prop(:,i);
    end
    
    Sigma_pred = zeros(nmu); %will fill
    
    sigma_hover = [2.728282836785659e-01; %Standard deviation of acceleration while hovering in place
                    2.011128064779450e-01;
                    5.632890328326323e-01];
            
    
    %R_t = diag(0.05*ones(nmu,1)); %Naive approach from paper
    %R_t = diag(10*ones(nmu,1)); %dont trust IMU at all
    %R_t = diag(zeros(nmu,1)); %Trust it a lot. Does this get rid of chol problem?
    % a little more semi-empirical way:
    theta_offset = .5*pi/180;
    projection_error = 9.81*[sin(theta_offset),sin(theta_offset),1-cos(theta_offset)]';
    v_var_prev = v_var_prev + projection_error*dt;
    %v_var_prev = v_var_prev + (sigma_hover).^2*dt+.01;
    R_t = diag([0.01^2,0.01^2,0.01^2,v_var_prev',(.7*pi/180)^2,(.7*pi/180)^2,(.7*pi/180)^2]); %TODO: figure out how to make this end up being a positive definite
    %R_t = zeros(nmu);
        
    for i = 1:2*nmu+1
        Sigma_pred = Sigma_pred + wc(i)*(sigma_points_prop(:,i)-mu_bar)*(sigma_points_prop(:,i)-mu_bar)';
    end
    Sigma_pred = Sigma_pred+R_t; %only add R_t once
    
            % this was found at https://robotics.stackexchange.com/questions/2000/maintaining-positive-definite-property-for-covariance-in-an-unscented-kalman-fil
            % as a way of fixing the positive definite covariance matrix
            % problem
            %Sigma_pred = .5*Sigma_pred+.5*Sigma_pred' + .00001*eye(nmu);
            
            % more links for reference:
            % https://stats.stackexchange.com/questions/48188/unscented-kalman-filter-negative-covariance-matrix
            % https://stats.stackexchange.com/questions/6364/making-square-root-of-covariance-matrix-positive-definite-matlab/6367#6367
            % https://www.researchgate.net/post/Guarantee_positiv_definite_P_in_an_UKF
    
    sigma_points_pred = get_sigma_points(mu_bar,Sigma_pred);
    
 
    %% Correction Step
    
    if time(t) > timeCamA(index_april)
        if method == 0 %normal method, not correcting for velocity
            if tagDetected(index_april)
                apriltag_detections = apriltag_detections+1;
                Z_t = [x_tag(index_april), y_tag(index_april), z_tag(index_april)]';
                z_prop = zeros(length(Z_t),2*nmu+1);
                % 	5.1	Calculate predicted measurement points
                % 	5.2 Calculate mean predicted measurement
                z_bar = zeros(length(Z_t),1);
                for i = 1:2*nmu+1
                    z_prop(:,i) = calc_meas_prediction(sigma_points_pred(:,i),Z_t);
                    z_bar = z_bar + wm(i)*z_prop(:,i);
                end

                % 	5.3	Calculate measurement model uncertainty
                % 	5.5	Calculate cross-covariance
                S_t = zeros(length(Z_t));
                %Sigma_z = diag([.05,.05,.05]);
                Sigma_z = diag([.0131^2,.0231^2,.0231^2]);
                Sigma_mu_z = zeros([nmu,length(Z_t)]);
                for i = 1:2*nmu+1
                    S_t = S_t + wc(i)*(z_prop(:,i)-z_bar)*(z_prop(:,i)-z_bar)';
                    Sigma_mu_z = Sigma_mu_z + wc(i)*(sigma_points_pred(:,i)-mu_bar)*(z_prop(:,i)-z_bar)';
                end
                S_t = S_t + Sigma_z; %only add Sigma_z once

                % 	5.6	Calculate Kalman Gain
                K_t = Sigma_mu_z/S_t; % 9 by 3

                % 	5.7	Calculate state estimate & covariance
                mu_corr = mu_bar + K_t*(Z_t - z_bar);
                Sigma_corr = Sigma_pred - K_t*S_t*K_t';
                sigma_points_corr = get_sigma_points(mu_corr,Sigma_corr);

                % this was found at https://robotics.stackexchange.com/questions/2000/maintaining-positive-definite-property-for-covariance-in-an-unscented-kalman-fil
                % as a way of fixing the positive definite covariance matrix
                % problem
                %Sigma_corr = .5*Sigma_corr+.5*Sigma_corr' + .00001*eye(nmu);

                STATE_ESTIMATES(:,t) = mu_corr;
                SIGMA_POINTS(:,:,t) = sigma_points_corr;
                SIGMA(:,:,t) = Sigma_corr;


            else % no tag detected in current frame
                STATE_ESTIMATES(:,t) = mu_bar;
                SIGMA_POINTS(:,:,t) = sigma_points_pred;
                SIGMA(:,:,t) = Sigma_pred;
            end
        end

        if method == 1  %correct for velocity
            if tagDetected(index_april)
                if rem(index_april,corrfreq) == 0 %run a correction step
                    if lastdetection == 0 %if a tag hasnt been detected yet, simply record the time and coordinates for safe keeping (b/c velocity can not be calculated)
                        lastdetection = time(t); %save last tag detected time
                        lastcoordinates = [x_tag(index_april), y_tag(index_april), z_tag(index_april)]'; %save last coordinates seen
                        STATE_ESTIMATES(:,t) = mu_bar;
                        SIGMA_POINTS(:,:,t) = sigma_points_pred;
                        SIGMA(:,:,t) = Sigma_pred;
                    else
                        %apriltag_detections = apriltag_detections+1;
                        tag_velocity = ([x_tag(index_april), y_tag(index_april), z_tag(index_april), ]' - lastcoordinates)*(time(t) - lastdetection); %this is the tags velocity in the cmera frame
                        Z_t = [x_tag(index_april), y_tag(index_april), z_tag(index_april),tag_velocity']';
                        z_prop = zeros(length(Z_t),2*nmu+1);
                        % 	5.1	Calculate predicted measurement points
                        % 	5.2 Calculate mean predicted measurement
                        z_bar = zeros(length(Z_t),1);
                        for i = 1:2*nmu+1
                            z_prop(:,i) = calc_meas_prediction2(sigma_points_pred(:,i));
                            z_bar = z_bar + wm(i)*z_prop(:,i);
                        end
                        v_var_prev = .01; % velocity is recallibrated, so get rid of running error

                        % 	5.3	Calculate measurement model uncertainty
                        % 	5.5	Calculate cross-covariance
                        S_t = zeros(length(Z_t));
                        %Sigma_z = diag([.05,.05,.05,.1,.1,.1]);
                        Sigma_z = diag([.0131^2,.0231^2,.0231^2,sqrt(2)*.0231^2,sqrt(2)*.0231^2,sqrt(2)*.0231^2]);
                        Sigma_mu_z = zeros([nmu,length(Z_t)]);
                        for i = 1:2*nmu+1
                            S_t = S_t + wc(i)*(z_prop(:,i)-z_bar)*(z_prop(:,i)-z_bar)';
                            Sigma_mu_z = Sigma_mu_z + wc(i)*(sigma_points_pred(:,i)-mu_bar)*(z_prop(:,i)-z_bar)';
                        end
                        S_t = S_t + Sigma_z; %only add Sigma_z once

                        % 	5.6	Calculate Kalman Gain
                        K_t = Sigma_mu_z/S_t; % 9 by 3

                        % 	5.7	Calculate state estimate & covariance
                        mu_corr = mu_bar + K_t*(Z_t - z_bar);
                        Sigma_corr = Sigma_pred - K_t*S_t*K_t';
                        sigma_points_corr = get_sigma_points(mu_corr,Sigma_corr);

                        % this was found at https://robotics.stackexchange.com/questions/2000/maintaining-positive-definite-property-for-covariance-in-an-unscented-kalman-fil
                        % as a way of fixing the positive definite covariance matrix
                        % problem
                        %Sigma_corr = .5*Sigma_corr+.5*Sigma_corr' + .00001*eye(nmu);

                        lastdetection = time(t); %save last tag detected time
                        lastcoordinates = [x_tag(index_april), y_tag(index_april), z_tag(index_april)]'; %save last coordinates seen

                        STATE_ESTIMATES(:,t) = mu_corr;
                        SIGMA_POINTS(:,:,t) = sigma_points_corr;
                        SIGMA(:,:,t) = Sigma_corr;
                    end
                else
                    STATE_ESTIMATES(:,t) = mu_bar;
                    SIGMA_POINTS(:,:,t) = sigma_points_pred;
                    SIGMA(:,:,t) = Sigma_pred;
                end
        
            end
            
        end
        index_april = index_april+1;
    else % not on timestep of video frame
            STATE_ESTIMATES(:,t) = mu_bar;
            SIGMA_POINTS(:,:,t) = sigma_points_pred;
            SIGMA(:,:,t) = Sigma_pred;
    end

%% Update Variables
mu_prev = STATE_ESTIMATES(:,t);
sigma_points_prev = SIGMA_POINTS(:,:,t);
Sigma_prev = SIGMA(:,:,t);
end
%%
%%%%%%%%%%%%% Supporting Functions %%%%%%%%%%%%%%%%%%%%%%%

function [sigma_points] = get_sigma_points(mu,Sigma)
% This function generates 2n+1 sigma points give the mean state and
% covariance of the state
global lambda nmu
Sigma = validateCovMatrix(Sigma);
sigma_points = [mu,mu + chol((nmu+lambda)*Sigma),mu - chol((nmu+lambda)*Sigma)];

end

%% Motion model

function [state_prop] = propagate_state(state_prev,U_t,dt)
% this function uses the motion model to calculate the propagated sigma
% points

a_l = U_t(1:3);
roll_meas = U_t(4);
pitch_meas = U_t(5);
yaw_meas = U_t(6);


DCM = angle2dcm(yaw_meas, pitch_meas, roll_meas);
a_g = DCM'*a_l; %rotate to global coordinate system
a_g(3) = a_g(3)-9.81; %correct for gravity in z direction
pos = state_prev(1:3)+ state_prev(4:6).*dt;
v = state_prev(4:6)+ a_g.*dt;
angles = U_t(4:6);

state_prop = [pos;v;angles];

end

%% Measurement Model

function [z_prop] = calc_meas_prediction(state_pred)
% this function calculates a predicted measurement from the predicted state
    global tag_coords
    DCM_CAM = angle2dcm(0,.19,0);
    DCM = angle2dcm(state_pred(9), state_pred(8), state_pred(7));
    z_prop = DCM_CAM'*(DCM*(tag_coords-state_pred(1:3)));
end

function [z_prop] = calc_meas_prediction2(state_pred)
% this function calculates a predicted measurement from the predicted state
    global tag_coords
    DCM_CAM = angle2dcm(0,.19,0);
    DCM = angle2dcm(state_pred(9), state_pred(8), state_pred(7));
    z_pos_prop = DCM_CAM'*(DCM*(tag_coords-state_pred(1:3)));
    z_vel_prop = DCM_CAM'*(DCM*(-state_pred(4:6))); %predicted measurement of tag velocity
    z_prop = [z_pos_prop;z_vel_prop];
end

% from https://stats.stackexchange.com/questions/6364/making-square-root-of-covariance-matrix-positive-definite-matlab/6367#6367
function [sigma] = validateCovMatrix(sig)

% [sigma] = validateCovMatrix(sig)
%
% -- INPUT --
% sig:      sample covariance matrix
%
% -- OUTPUT --
% sigma:    positive-definite covariance matrix
%

EPS = 10^-6;
ZERO = 10^-10;

sigma = sig;
[~, err] = cholcov(sigma, 0);

if (err ~= 0)
    % the covariance matrix is not positive definite!
    [v, d] = eig(sigma);

    % set any of the eigenvalues that are <= 0 to some small positive value
    for n = 1:size(d,1)
        if (d(n, n) <= ZERO)
            d(n, n) = EPS;
        end
    end
    % recompose the covariance matrix, now it should be positive definite.
    sigma = v*d*v';

    [~, err] = cholcov(sigma, 0);
    if (err ~= 0)
        disp('ERROR!');
    end
end
end

