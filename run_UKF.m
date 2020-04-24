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

wm = [lamba/(nmu+lambda);1/(2*(nmu+lambda))*ones(2*nmu,1)];
wc = [lamba/(nmu+lambda)+(1-a^2+b);1/(2*(nmu+lambda))*ones(2*nmu,1)];

% Coordinates of tag(s) in global frame
% Tag order: Tag36h11
tag_coords = [2.8448 0 -1.4224]';


% Load Data

filename = '/Users/kevinshoyer/Desktop/DJITelloPy_E205.nosync/AprilTag/apriltag-master/python/logs/Tello_Log_2020_04_23_17_40_10.csv';
filename_april = '/Users/kevinshoyer/Desktop/DJITelloPy_E205.nosync/AprilTag/apriltag-master/python/logs/Tello_Log_2020_04_23_17_40_10_april.csv';

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
y_tag = dataArray{:, 4}; % in our local coordinate system
z_tag = dataArray{:, 5};
x_tag = dataArray{:, 6};
clearvars filename_april delimiter formatSpec fileID dataArray ans;

t_diff = 23.72-21.1;

timeCamA = timeCam+t_diff;


% Covariance matrix for the control data, calculated in EKF_setup
Sigma_u = ones(6,6); % ax,ay,az,roll_meas,pitch_meas,yaw_meas

% Covariance matrix for the measurement data, calculated in EKF_setup
Sigma_z = ones(3,3); %x_tag_l,y_tag_l,z_tag_l


%% Callibrate accelerometers using stationary data. This gets rid of offsets due to misallignment of accelerometer

% callibration procedure will occur when the vehicle is on a flat surface

[indices_real_data,accels] = find(a_z ~= -1); % finds indices and values for hieghts not equal to zero
[takeoff_point,accels] = find(a_z > 10.5); % finds indices and values for hieghts not equal to zero
start_data= indices_real_data(1);
takeoff = takeoff_point(1); % index for when the drone takes off. Callibration procedures will end

% calculate offsets during callibration procedure
ax_bias = mean(a_x(start_data:takeoff));
ay_bias = mean(a_y(start_data:takeoff));
az_bias = mean(a_z(start_data:takeoff));

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

Sigma_prev = diag([1,1,1,1,1,1,1,1,1]); % maybe adjust these

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

index_april = 1; % index to track video frame

for t = (takeoff-1):length(time)
    %% Set variables for loop
    dt = time(t)-time(t-1);
    U_t = [a_x(t), a_y(t), a_z(t), roll(t), pitch(t), yaw(t)]';
    
    %% Prediction Step
    %global nmu wm
    %Calculate the predicted state by propegating through the motion model
    
    sigma_points_prop = zeros(nmu,2*nmu+1);
    mu_bar = zeros(nmu,1);
    
    for i = 1:2*nmu+1
        sigma_points_prop(:,i) = propagate_state(sigma_points_prev(:,i),U_t,dt);
        mu_bar = mu_bar + wm(i)*sigma_points_prop(:,i);
    end
    
    Sigma_pred = zeros(nmu);
    R_t = diag(0.05*ones(nmu,1)); %TODO: get uncertainties for IMU
    
    % TODO: what the fuck is this math/dimensionality
    for i = 1:2*nmu+1
        Sigma_pred = Sigma_pred + wc(i)*(sigma_points_prop(:,i)-mu_bar)*(sigma_points_prop(:,i)-mu_bar)' + R_t;
    end
    
    sigma_points_pred = get_sigma_points(mu_bar,Sigma_pred);
    
    %% Correction Step
    
    %global tag_coords
    
    if time(t) > timeCamA(index_april)
        if tagDetected(index_april)
            Z_t = [x_tag, y_tag, z_tag]';
            
            z_prop = zeros(nmu,2*nmu+1);
            % 	5.1	Calculate predicted measurement points
            % 	5.2 Calculate mean predicted measurement
            for i = 1:2*nmu+1
                z_prop(:,i) = calc_meas_prediction(sigma_points_pred(:,i),Z_t);
                z_bar = z_bar + wm(i)*z_prop(:,i);
            end
            
            % 	5.3	Calculate measurement model uncertainty
            % 	5.5	Calculate cross-covariance
            S_t = zeros(length(Z_t));
            Sigma_z = 0.10*eye(length(Z_t)); % TODO: make this more legit, prob not a diag
            Sigma_mu_z = zeros([nmu,length(Z_t)]);
            for i = 1:2*nmu+1
                S_t = S_t + + wc(i)*(z_prop(:,i)-z_bar)*(z_prop(:,i)-z_bar)' + Sigma_z;
                Sigma_mu_z = Sigma_mu_z + wc(i)*(sigma_points_pred(:,i)-mu_bar)*(z_prop(:,i)-z_bar)';
            end
            
            % 	5.6	Calculate Kalman Gain
            K_t = Sigma_mu_z * S_t^-1;
            
            % 	5.7	Calculate state estimate & covariance
            mu_corr = mu_bar + K_t*(Z_t - z_bar);
            Sigma_corr = Sigma_pred - K_t*S_t*K_t';
            sigma_points_corr = get_sigma_points(mu_corr,Sigma_corr);
            
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

% figure(1)
% plot(STATE_ESTIMATES(1,:),STATE_ESTIMATES(2,:));
% hold on
% plot(x_gps,y_gps,'x')
% xlabel('X position (meters)')
% ylabel('y position (meters)')
% title('state estimate of position')

% Motion plot with covariance ellipses


figure(2)
% xlim([xmin ,xmax])
% ylim([ymin,ymax])
for i = 250:length(time)
    xmax = STATE_ESTIMATES(1,i)+1;
    xmin = STATE_ESTIMATES(1,i)-9;
    ymax = STATE_ESTIMATES(2,i)+5;
    ymin = STATE_ESTIMATES(2,i)-5;
    hold all
    xlim([xmin ,xmax])
    ylim([ymin,ymax])
    hold on
    plot(STATE_ESTIMATES(1,1:i),STATE_ESTIMATES(2,1:i),'.-k')
    hold on
    plot(x_gps(i),y_gps(i),'xb')
    hold on
    if rem(i,10) == 0
        r_ellipse = cov_ellipse(STATE_ESTIMATES(1:2,i),SIGMA(1:2,1:2,i));
        plot(r_ellipse(:,1),r_ellipse(:,2),'-r')
        hold on
    end
%     if rem(i,10) == 9
%         r_ellipse = cov_ellipse(STATE_PREDICTIONS(1:2,i),PREDICTION_SIGMA(1:2,1:2,i));
%         plot(STATE_PREDICTIONS(1,i),STATE_PREDICTIONS(2,i))
%         plot(r_ellipse(:,1),r_ellipse(:,2),'-r')
%         hold on
%     end
    hold on
    pause(.01)
    xlabel('x position (m)')
    ylabel('y position (m)')
    title('State Estimate with Covariance Ellipse')
    legend('State Estimate','raw_gpsdata','95% confidence interval')
    hold off
end

%%
%%%%%%%%%%%%% Supporting Functions %%%%%%%%%%%%%%%%%%%%%%%

function [sigma_points] = get_sigma_points(mu,Sigma)

global lambda nmu

sigma_points = [mu,mu + chol((nmu+lambda)*Sigma),mu - chol((nmu+lambda)*Sigma)];

end



%% Motion model

function [state_prop] = propagate_state(state_prev,U_t,dt)
% this function uses the motion model to calculate the propagated sigma
% point

a_l = U_t(1:3)';
roll_meas = U_t(4);
pitch_meas = U_t(5);
yaw_meas = U_t(6);


DCM = angle2dcm(yaw_meas, pitch_meas, roll_meas);
a_g = DCM*a_l; %rotate to global coordinate system
a_g(3) = a_gi(3)-9.81; %correct for gravity in z direction
pos = state_prev(1:3)'+ state_prev(4:6)'.*dt;
v = state_prev(4:6)'+ a_g.*dt;
angles = U_t(7:9); % TODO: think about this one later

state_prop = [pos;v;angles];

end

%% Measurement Model

function [z_prop] = calc_meas_prediction(state_pred,Z_t)
% this function calculates a predicted measurement from the predicted state
    global tag_coords
    DCM = angle2dcm(state_pred(9), state_pred(8), state_pred(7));
    z_prop = DCM'*(tag_coords-state_pred(1:3));
end

%% Import Data
function importfile(fileToRead1)
%IMPORTFILE(FILETOREAD1)
%  Imports data from the specified file
%  FILETOREAD1:  file to read

%  Auto-generated by MATLAB on 01-Apr-2020 13:09:34

% Import the file
newData1 = load('-mat', fileToRead1);

% Create new variables in the base workspace from those fields.
vars = fieldnames(newData1);
for i = 1:length(vars)
    assignin('base', vars{i}, newData1.(vars{i}));
end
end


%% Confidence Elipse
function [r_ellipse] = cov_ellipse(state,covariance)

% Calculate the eigenvectors and eigenvalues of covariance matrix.
[eigenvec, eigenval ] = eig(covariance);

% Get the index of the largest eigenvector
[largest_eigenvec_ind_c, r] = find(eigenval == max(max(eigenval)));
largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);

% Get the largest eigenvalue
largest_eigenval = max(max(eigenval));

% Get the smallest eigenvector and eigenvalue
if(largest_eigenvec_ind_c == 1)
    smallest_eigenval = max(eigenval(:,2));
    smallest_eigenvec = eigenvec(:,2);
else
    smallest_eigenval = max(eigenval(:,1));
    smallest_eigenvec = eigenvec(1,:);
end

% Calculate the angle between the x-axis and the largest eigenvector
angle = atan2(largest_eigenvec(2), largest_eigenvec(1));

% Shift the angle from 0 to 2pi for plotting purposes
if(angle < 0)
    angle = angle + 2*pi;
end


% Get the 95% confidence interval error ellipse
chisquare_val = 2.4477;
theta_grid = linspace(0,2*pi);
phi = angle;
X0=state(1);
Y0=state(2);
a1=chisquare_val*sqrt(largest_eigenval);
b1=chisquare_val*sqrt(smallest_eigenval);

% the ellipse in x and y coordinates 
ellipse_x_r  = a1*cos( theta_grid );
ellipse_y_r  = b1*sin( theta_grid );

%rotation matrix
R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];

%rotate the ellipse to some angle phi
r_ellipse = [ellipse_x_r;ellipse_y_r]' * R + [X0;Y0]';

end