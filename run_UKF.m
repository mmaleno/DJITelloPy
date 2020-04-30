%% Unscented Kalman Filter
% Max Maleno and Kevin Shoyer
% 4/23/2020

%ADD DESCRIPTION HERE

clear all
clear figure

global a b k lambda nmu wm wc tag_coords

a = 0.75; %alpha, controls spread of sigma points
b = 2; %beta, 2 is best for gaussian state distributions
k = 0; %commonly set to zero
nmu = 9; %number of states in state vector
lambda = a^2*(nmu+k)-nmu;

wm = [lambda/(nmu+lambda);1/(2*(nmu+lambda))*ones(2*nmu,1)];
wc = [lambda/(nmu+lambda)+(1-a^2+b);1/(2*(nmu+lambda))*ones(2*nmu,1)];

% Coordinates of tag(s) in global frame
% Tag order: Tag36h11
tag_coords = [2.74,0.29,-0.67]';%for linear test


% Load Data

% filename = '/Users/kevinshoyer/Desktop/DJITelloPy_E205.nosync/AprilTag/apriltag-master/python/logs/Tello_Log_2020_04_23_17_40_10.csv';
% filename_april = '/Users/kevinshoyer/Desktop/DJITelloPy_E205.nosync/AprilTag/apriltag-master/python/logs/Tello_Log_2020_04_23_17_40_10_april.csv';
% filename = '/Users/kevinshoyer/Desktop/DJITelloPy_E205.nosync/AprilTag/apriltag-master/python/logs/Tello_Log_2020_04_24_16_20_52.csv';
% filename_april = '/Users/kevinshoyer/Desktop/DJITelloPy_E205.nosync/AprilTag/apriltag-master/python/logs/Tello_Log_2020_04_24_16_20_52_april.csv';

% filename = '/Users/kevinshoyer/Desktop/DJITelloPy_E205.nosync/AprilTag/apriltag-master/python/logs/Tello_Log_2020_04_27_19_30_26.csv';
% filename_april = '/Users/kevinshoyer/Desktop/DJITelloPy_E205.nosync/AprilTag/apriltag-master/python/logs/Tello_Log_2020_04_27_19_30_26_april.csv';

filename = '/Users/kevinshoyer/Desktop/DJITelloPy_E205.nosync/clean_trials/Linear_imu.csv';
filename_april = '/Users/kevinshoyer/Desktop/DJITelloPy_E205.nosync/clean_trials/Linear_april.csv';
delimiter = ',';

formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

fileID = fopen(filename,'r');

dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter,  'ReturnOnError', false);

% Close the text file.
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
y_tag = dataArray{:, 4}*0.6096/.5877; % in our local coordinate system
z_tag = dataArray{:, 5}*0.6096/.5877;
x_tag = dataArray{:, 6}*0.6096/.5877;
clearvars filename_april delimiter formatSpec fileID dataArray ans;

timeCamA = timeCam+(time(1)-timeCam(1));

%% Callibrate accelerometers using stationary data. This gets rid of offsets due to misallignment of accelerometer

% callibration procedure will occur when the vehicle is on a flat surface
[start_data,time_calstart] = find(time>36.5);
[takeoff,time_calend] = find(time>38);

% [indices_real_data,accels] = find(a_z ~= -1); % finds indices and values for hieghts not equal to zero
% [takeoff_point,accels] = find(a_z > 10.5); % finds indices and values for hieghts not equal to zero
% start_data= indices_real_data(1);
% takeoff = takeoff_point(1); % index for when the drone takes off. Callibration procedures will end

% calculate offsets during callibration procedure
ax_bias = mean(a_x(start_data(1):takeoff(1)));
ay_bias = mean(a_y(start_data(1):takeoff(1)));
az_bias = mean(a_z(start_data(1):takeoff(1)))-9.81; %assuming z is alligned with gravity

% delete these offsets from data
a_x = a_x - ax_bias;
a_y = a_y - ay_bias;
a_z = a_z - az_bias;

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

for t = (takeoff-1):length(time)
%for t = (takeoff-1):1148
    %% Set variables for loop
    dt = time(t)-time(t-1);
    U_t = [a_x(t), a_y(t), a_z(t), roll(t), pitch(t), yaw(t)]';
    
    %% Prediction Step
    %global nmu wm
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
                
    v_var_prev = v_var_prev + (sigma_hover).^2*dt+.01;
    
    %R_t = diag(0.05*ones(nmu,1)); %Naive approach from paper
    
    %R_t = diag(10*ones(nmu,1)); %dont trust IMU at all
    
    %R_t = diag(zeros(nmu,1)); %Trust it a lot. Does this get rid of chol problem?
    
    % a little more semi-empirical way:
    
    R_t = diag([0.01,0.01,0.01,v_var_prev',0.01,0.01,0.01]); %TODO: figure out how to make this end up being a positive definite
     %R_t = [
    %R_t = zeros(nmu);
        
    for i = 1:2*nmu+1
        Sigma_pred = Sigma_pred + wc(i)*(sigma_points_prop(:,i)-mu_bar)*(sigma_points_prop(:,i)-mu_bar)';
    end
    Sigma_pred = Sigma_pred+R_t; %only add R_t once
    
                % this was found at https://robotics.stackexchange.com/questions/2000/maintaining-positive-definite-property-for-covariance-in-an-unscented-kalman-fil
            % as a way of fixing the positive definite covariance matrix
            % problem
            Sigma_pred = .5*Sigma_pred+.5*Sigma_pred' + .00001*eye(nmu);
    
    sigma_points_pred = get_sigma_points(mu_bar,Sigma_pred);
    
    %% Correction Step
    
    %global tag_coords
    
    if time(t) > timeCamA(index_april)
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
            Sigma_z = diag([.05,.05,.05]);
            Sigma_mu_z = zeros([nmu,length(Z_t)]);
            for i = 1:2*nmu+1
                S_t = S_t + wc(i)*(z_prop(:,i)-z_bar)*(z_prop(:,i)-z_bar)';
                Sigma_mu_z = Sigma_mu_z + wc(i)*(sigma_points_pred(:,i)-mu_bar)*(z_prop(:,i)-z_bar)';
            end
            S_t = S_t + Sigma_z; %only add Sigma_z once
            
            % 	5.6	Calculate Kalman Gain
            %K_t = Sigma_mu_z*inv(S_t);
            K_t = Sigma_mu_z/S_t; % 9 by 3
            
            % 	5.7	Calculate state estimate & covariance
            mu_corr = mu_bar + K_t*(Z_t - z_bar);
            Sigma_corr = Sigma_pred - K_t*S_t*K_t';
            sigma_points_corr = get_sigma_points(mu_corr,Sigma_corr);
            
            % this was found at https://robotics.stackexchange.com/questions/2000/maintaining-positive-definite-property-for-covariance-in-an-unscented-kalman-fil
            % as a way of fixing the positive definite covariance matrix
            % problem
            Sigma_corr = .5*Sigma_corr+.5*Sigma_corr' + .00001*eye(nmu);
            
            STATE_ESTIMATES(:,t) = mu_corr;
            SIGMA_POINTS(:,:,t) = sigma_points_corr;
            SIGMA(:,:,t) = Sigma_corr;
            

        else % no tag detected in current frame
            STATE_ESTIMATES(:,t) = mu_bar;
            SIGMA_POINTS(:,:,t) = sigma_points_pred;
            SIGMA(:,:,t) = Sigma_pred;
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



%%%%%%%%%%%%%%%%%% Plotting %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% Plotting %%%%%%%%%%%%%%%%%%%%%
% 
% 
% % Motion plot with covariance ellipses
% figure(1)
%     window = .5; % used to update moving window
%     for i = 1:length(time)
% 
%         %live plot of the covariance ellipsiod and the state estimate 
%         subplot1 = subplot(2,4,4);
% 
%             %update plotting window
%             xmax = STATE_ESTIMATES(1,i)+window;
%             xmin = STATE_ESTIMATES(1,i)-window;
%             ymax = STATE_ESTIMATES(2,i)+window;
%             ymin = STATE_ESTIMATES(2,i)-window;
%             zmax = STATE_ESTIMATES(3,i)+window;
%             zmin = STATE_ESTIMATES(3,i)-window;
%             
%             xlim(subplot1,[xmin ,xmax])
%             ylim(subplot1,[ymin,ymax])
%             zlim(subplot1,[zmin,zmax])
%             view_axis = [1,1,1]; % vector pointing from origin to camera
%             view(subplot1,view_axis); % set to isometric view
% 
%             plot3(STATE_ESTIMATES(1,1:i),STATE_ESTIMATES(2,1:i),STATE_ESTIMATES(3,1:i),'.-k','Parent',subplot1) % plots the state estimate
%             hold on
%             
% 
%             %ploting covariance elipse only every 20th time step 
%             if rem(i,20) == 0
%                 xy_r_ellipse = cov_ellipse(STATE_ESTIMATES(1:2,i),SIGMA(1:2,1:2,i)); % covariance ellipse on xy plane
%                 yz_r_ellipse = cov_ellipse(STATE_ESTIMATES(2:3,i),SIGMA(2:3,2:3,i)); % covariance ellipse on yz plane
%                 state_xz = [STATE_ESTIMATES(1,i),STATE_ESTIMATES(3,i)]; % state vector for X = [x:z]
%                 sig_xz = [SIGMA(1,1,i),SIGMA(1,3,i);SIGMA(3,1,i),SIGMA(3,3,i)]; % covariance matrix for X = [x;z]
%                 xz_r_ellipse = cov_ellipse(state_xz,sig_xz); % covariance ellipse on xz plane
%                 plot3(xy_r_ellipse(:,1),xy_r_ellipse(:,2),ones(1,length(xy_r_ellipse(:,2)))*STATE_ESTIMATES(3,i),'-r','Parent',subplot1) % plots the covariance ellipse on the x,y plane
%                 hold on
%                 plot3(ones(1,length(yz_r_ellipse(:,2)))*STATE_ESTIMATES(1,i),yz_r_ellipse(:,1),yz_r_ellipse(:,2),'-r','Parent',subplot1) % plots the covariance ellipse on the y,z plane
%                 hold on 
%                 plot3(xz_r_ellipse(:,1),ones(1,length(xz_r_ellipse(:,2)))*STATE_ESTIMATES(2,i),xz_r_ellipse(:,2),'-r','Parent',subplot1) % plots the covariance ellipse on the x,z plane
%                 hold on
%             end
% 
%              xlabel('x position (m)')
%              ylabel('y position (m)')
%              zlabel('z position (m)')
%              title('Real Time State Estimate with Covariance Ellipsiod')
% 
% 
%          subplot2 = subplot(2,4,[1,3]);
%              % note that these axis limits created below do not create a cube,
%              % so the relationship between x,y and z are not spacially
%              % equivalent. However, when a cube is chosen, you can not see the
%              % state estimate clearly (as the gps measurements are in the way
%              xmax1 = max(STATE_ESTIMATES(1,:))+1;
%              xmin1 = min(STATE_ESTIMATES(1,:))-1;
%              ymax1 = max(STATE_ESTIMATES(2,:))+1;
%              ymin1 = min(STATE_ESTIMATES(2,:))-1;
%              zmax1 = max(STATE_ESTIMATES(3,:))+1;
%              zmin1 = min(STATE_ESTIMATES(3,:))-1;
% 
%              % if you want a spacially accurate plotting window, use this:
% %              xmax1 = 30;
% %              xmin1 = -30;
% %              ymax1 = 10;
% %              ymin1 = -50;
% %              zmax1 = 45;
% %              zmin1 = -15;
%              xlim(subplot2,[xmin1 ,xmax1])
%              ylim(subplot2,[ymin1,ymax1])
%              zlim(subplot2,[zmin1,zmax1])
%              view_axis = [1,1,1]; % vector pointing from origin to camera
%              view(subplot2,view_axis); % set to isometric view
% 
%              plot3(STATE_ESTIMATES(1,:),STATE_ESTIMATES(2,:),STATE_ESTIMATES(3,:),'.-k','Parent',subplot2) % plot state estimate positions for all time
%              hold on
%              plot3(STATE_ESTIMATES(1,i),STATE_ESTIMATES(2,i),STATE_ESTIMATES(3,i),'or','Parent',subplot2) % highlight current state estimate
%              hold on
%              
% 
%              xlabel('x position (m)')
%              ylabel('y position (m)')
%              zlabel('z position (m)')
%              title('Full Path State Estimate Position')
%              legend('Location','northeast')
%              legend('Full State Estimate Path','Previous State Estiates','Raw GPS Measurements')
% 
%          subplot3 = subplot(2,4,[5,7]);
%              yaw_std = squeeze(sqrt(SIGMA(9,9,:))); % squeeze gets rid of extra dimensions, sqrt because we want to plot standard deviation
%              errorbar(time,STATE_ESTIMATES(9,:),yaw_std,'.k')
%              hold on
%     %          plot(time,yaw,'xb') % too messy to plot raw data too
% 
%              xlabel('Time (s)')
%              ylabel('Angle (rad)')
%              title('Yaw State Estimate with Standard Deviation Error Bounds')
% 
%           subplot4 = subplot(2,4,8);
%                      %plot(time,STATE_ESTIMATES(9,:),'.k')
%              hold on
%              yaw_std = squeeze(sqrt(SIGMA(4,4,:))); % squeeze gets rid of extra dimensions, sqrt because we want to plot standard deviation
%              errorbar(time(1:20),STATE_ESTIMATES(4,1:20),yaw_std(1:20),'.k')
%              hold on
%     %          plot(time,yaw,'xb') % too messy to plot raw data too
% 
%              xlabel('Time (s)')
%              ylabel('Angle (rad)')
%              title(' Zoomed in Yaw State Estimate')
% 
% 
%          pause(.01) % pause for real time path
%          %hold off
%     end

%%
%%%%%%%%%%%%% Supporting Functions %%%%%%%%%%%%%%%%%%%%%%%

function [sigma_points] = get_sigma_points(mu,Sigma)
% This function generates 2n+1 sigma points give the mean state and
% covariance of the state
global lambda nmu
Sigma(Sigma < 0) = 0;
Sigma
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

function [z_prop] = calc_meas_prediction(state_pred,Z_t)
% this function calculates a predicted measurement from the predicted state
    global tag_coords
    DCM_CAM = angle2dcm(0,.19,0);
    DCM = angle2dcm(state_pred(9), state_pred(8), state_pred(7));
    z_prop = DCM_CAM'*(DCM*(tag_coords-state_pred(1:3)));
end

%% Confidence Elipse
function [r_ellipse] = cov_ellipse(state,covariance)
%this function outputs the x,y vectors of the covariance ellipse for a
%given state estimate and covariance (2 DIMENSIONS)

% An eigenvector is a vector that does not change through a linear
% transformation.  In an ellipse (tilted), the major and minor axis do not
% change through a linear transformation. Therefor, the major and minor
% axis correspond to the eigenvectors of the covariance matrix.
% Furthermore, the corresponding eigenvalue explains the length of the
% eigenvector. For an ellipse, this is the length of the major and minor
% axis.

% procedure:
%     1) find the eigenvalue and eigenvectors for the covariance matrix.
%     These are the directions and lengths of the major and minor axis of
%     the covariance ellipse 
%     2) find the angle that the covariance ellipse is tilted. This is the
%     angle between the x-axis and major axis of ellipse
%     3) Scale the length of the major and minor axis to match the desired
%     confidence ellipse using chi-squared parameter for a given confidence
%     4) create an ellipse with propper major and minor axis
%     5) Tilt the ellipse by theta calculated above and shift the centroid
%     up to the the mean state estimate

% Calculate the eigenvectors and eigenvalues of the covariance matrix.
[eigenvec, eigenval ] = eig(covariance);

% Reorder the eigenvalues in ascending order. Because the order of the eig output is not always in ascending
%     order (although it ussually is), it is recommended in matlab help
%     center for "eig" to mannually find the eigenvalues and eigenvectors
%     you need.
[eigval,ind] = sort(diag(eigenval)); %extracts the eigenvalues from matrix and sorts them in ascending order

% now using these indices of this sort, we can reorder the corresponding
% eigenvectors
eigvec = eigenvec(:,ind);

%define the major and minor axis of the ellipse
major_axis = eigvec(:,2);
major_scale = eigval(2);

minor_axis = eigvec(:,1);
minor_scale = eigval(1);

% calculate the angle between the x axis and the major axis of the ellipse
angle = atan2(major_axis(2),major_axis(1));

% Shift the angle from 0 to 2pi for plotting purposes
if(angle < 0)
     angle = angle + 2*pi;
end

% now stretch the confidence ellipse to match the desired confidence
% Get the 95% confidence interval error ellipse

% chi square values for DOF = 2:
%       Confidence              chi squared value
%           90                          4.605
%           95                          5.991
%           99                          9.210


chisquare_val = 9.210;
theta_grid = linspace(0,2*pi); % set up points to plot

% major axis and minor axis are scaled by the chi-squared value. They are
% then square rooted to correspond to a scaled standard deviation value
% rather than a variance value
a1=sqrt(chisquare_val*major_scale); % major axis / 2
b1=sqrt(chisquare_val*minor_scale); % minor axis / 2

% the ellipse in x and y coordinates 
ellipse_x_r  = a1*cos( theta_grid );
ellipse_y_r  = b1*sin( theta_grid );

%rotation matrix constructed to rotate the ellipse by the angle calculated
R = [ cos(angle) sin(angle); -sin(angle) cos(angle) ];

% the mean state estimate, used to shift the covariance ellipse to the
% propper place
X0=state(1);
Y0=state(2);

%rotate the ellipse and offset it to be centered at the state estimate
r_ellipse = [ellipse_x_r;ellipse_y_r]' * R + [X0;Y0]';

end