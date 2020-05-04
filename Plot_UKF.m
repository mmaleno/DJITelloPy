%%%%%%%%%%%%%%%%% Plotting %%%%%%%%%%%%%%%%%%%%%
% This script is used to plot the output of run_UKF.m
% Authors: Kevin Shoyer and Max Maleno
% Date: 5/1/2020

%Desired plots
% real time plot with videos going at same time
% some graph of performance (RMSE)
% static 2D plot with covariance ellipse
% plot of x states with confidence bar
% Plot of raw data (cam and accelerometer)

%% Load in Truth Data
%linear
% x_true = [0 0 2/.0254 0 3]* 0.0254;
% y_true = [0 0 0 0 0];
% z_true = [0 -32 -32 -32 0]*0.0254;

%rectangular

%complex
% x_true = [0 0 .50 .50 1.25 1.25 0 8*.0254];
% y_true = [0 0 0 .5 .5 .25 0 3*.0254];
% z_true = [0 -32 -32 -32 -32 -32 -32 0]*0.0254;

%large linear
x_true = [0 0 12];
y_true = [0 0 -3.5*12*.0254];
z_true = [0 -32 -32]*0.0254;


%% 3D Motion Plot

figure(24) %3D plot

persistance = 25;
delay = 0.05;
for i = 200:690
    %plot3(x_true,-y_true,-z_true,'b--') % plots the state estimate
%     hold on
    if i<persistance+1
        plot3(STATE_ESTIMATES(1,1:i),STATE_ESTIMATES(2,1:i),STATE_ESTIMATES(3,1:i),'o')
    else
        plot3(STATE_ESTIMATES(1,i-persistance:i),STATE_ESTIMATES(2,i-persistance:i), ...
                                            STATE_ESTIMATES(3,i-persistance:i),'o')
    end
    xlabel('x position (m)')
    ylabel('~y position (m)')
    zlabel('~z position (m)')
    axis([-10, 10, -10, 10, -10, 10])
    pause(delay)
    hold off
end
%% Static 3d Plot

figure(245)
plot3(x_true,-y_true,-z_true,'b--') % plots the state estimate
    hold on
plot3(STATE_ESTIMATES(1,1:find(time==lastdetection)),-STATE_ESTIMATES(2,1:find(time==lastdetection)),-STATE_ESTIMATES(3,1:find(time==lastdetection)),'o')
axis([-1.5, 1.5, -1.5, 1.5, -1, 1])
xlabel('x position (m)')
ylabel('~y position (m)')
zlabel('~z position (m)')

%% Plot of all States over time 
figure(25) %1D plot over time
subplot(3,1,1)
title('States Estimates over time')
plot(STATE_ESTIMATES(1,:))
hold on
plot(STATE_ESTIMATES(2,:))
hold on
plot(STATE_ESTIMATES(3,:))
hold on
ylabel('Position (m)')
legend('x','y','z')
subplot(3,1,2)
plot(STATE_ESTIMATES(4,:))
hold on
plot(STATE_ESTIMATES(5,:))
hold on
plot(STATE_ESTIMATES(6,:))
ylabel('velocities (m/s)')
legend('v_x','v_y','v_z')
subplot(3,1,3)
plot(STATE_ESTIMATES(7,:))
hold on
plot(STATE_ESTIMATES(8,:))
hold on
plot(STATE_ESTIMATES(9,:))
ylabel('Angle (rad)')
legend('Roll','Pitch','Yaw')
xlabel('Sample')

%% Plot of x states over time (and yaw) With error bars

figure(26) %1D plot over time
subplot(3,1,1)
title('X States Estimates over time')
errorbar(STATE_ESTIMATES(1,:),squeeze(sqrt(SIGMA(1,1,:))))
ylabel('X Position (m)')
subplot(3,1,2)
errorbar(STATE_ESTIMATES(4,:),squeeze(sqrt(SIGMA(4,4,:))))
ylabel('X velocity (m/s)')
subplot(3,1,3)
errorbar(STATE_ESTIMATES(9,:),squeeze(sqrt(SIGMA(9,9,:))))
ylabel('Yaw (rad)')
xlabel('Sample')

%% 3d plot with position covariance ellipsoids

%covariance ellipsoids 3Dplot
x_true = [0 0 2/.0254 0 3]* 0.0254;
y_true = [0 0 0 0 0];
z_true = [0 -32 -32 -32 0]*0.0254;

figure(1)
for i =1:length(STATE_ESTIMATES(1,1:2360))
    if rem(i,101) == 0
                xy_r_ellipse = cov_ellipse(STATE_ESTIMATES(1:2,i),SIGMA(1:2,1:2,i)); % covariance ellipse on xy plane
                yz_r_ellipse = cov_ellipse(STATE_ESTIMATES(2:3,i),SIGMA(2:3,2:3,i)); % covariance ellipse on yz plane
                state_xz = [STATE_ESTIMATES(1,i),STATE_ESTIMATES(3,i)]; % state vector for X = [x:z]
                sig_xz = [SIGMA(1,1,i),SIGMA(1,3,i);SIGMA(3,1,i),SIGMA(3,3,i)]; % covariance matrix for X = [x;z]
                xz_r_ellipse = cov_ellipse(state_xz,sig_xz); % covariance ellipse on xz plane
                plot3(xy_r_ellipse(:,1),xy_r_ellipse(:,2),ones(1,length(xy_r_ellipse(:,2)))*STATE_ESTIMATES(3,i),'-r') % plots the covariance ellipse on the x,y plane
                hold on
                plot3(ones(1,length(yz_r_ellipse(:,2)))*STATE_ESTIMATES(1,i),yz_r_ellipse(:,1),yz_r_ellipse(:,2),'-r') % plots the covariance ellipse on the y,z plane
                hold on 
                plot3(xz_r_ellipse(:,1),ones(1,length(xz_r_ellipse(:,2)))*STATE_ESTIMATES(2,i),xz_r_ellipse(:,2),'-r') % plots the covariance ellipse on the x,z plane
                hold on
                plot3(STATE_ESTIMATES(1,i),STATE_ESTIMATES(2,i),STATE_ESTIMATES(3,i),'o')
                plot3(STATE_ESTIMATES(1,:),STATE_ESTIMATES(2,:),STATE_ESTIMATES(3,:),'--')
                hold on
                plot3(x_true,y_true,z_true)
    end
end

%% 2D covariance in xy plot
figure(1)
    plot(STATE_ESTIMATES(1,1:2310),STATE_ESTIMATES(2,1:2310),'b')
    hold on
    plot(x_true,y_true,'--')
    hold on
% for i =1:length(STATE_ESTIMATES(1,1:2310))
%     if rem(i,99) == 0
%                 xy_r_ellipse = cov_ellipse(STATE_ESTIMATES(1:2,i),SIGMA(1:2,1:2,i)); % covariance ellipse on xy plane
%                 plot(xy_r_ellipse(:,1),xy_r_ellipse(:,2),'-r') % plots the covariance ellipse on the x,y plane
%                 hold on
%     end
% end
    legend('State Estimate','Truth Data','90% Confidence Ellipse')
    title('Top Down View of State Estimate')
    xlabel('x Position (m)')
    ylabel('y Position (m)')
%% Motion plot with covariance ellipsoids

%Calculate all covariance matrices before hand to improve plotting time
xy_r_ellipse = zeros(100,2,length(time));
yz_r_ellipse = zeros(100,2,length(time));
xz_r_ellipse = zeros(100,2,length(time));

for i = 1:length(time)
    if rem(i,40) == 0
                xy_r_ellipse(:,:,i) = cov_ellipse(STATE_ESTIMATES(1:2,i),SIGMA(1:2,1:2,i)); % covariance ellipse on xy plane
                yz_r_ellipse(:,:,i) = cov_ellipse(STATE_ESTIMATES(2:3,i),SIGMA(2:3,2:3,i)); % covariance ellipse on yz plane
                state_xz = [STATE_ESTIMATES(1,i),STATE_ESTIMATES(3,i)]; % state vector for X = [x:z]
                sig_xz = [SIGMA(1,1,i),SIGMA(1,3,i);SIGMA(3,1,i),SIGMA(3,3,i)]; % covariance matrix for X = [x;z]
                xz_r_ellipse(:,:,i) = cov_ellipse(state_xz,sig_xz); % covariance ellipse on xz plane
    end
end

figure(1)
    window = 1; % used to update moving window
    for i = 1:length(time)

        %live plot of the covariance ellipsiod and the state estimate 
        subplot1 = subplot(2,3,[4,5]);

            %update plotting window
            xmax = STATE_ESTIMATES(1,i)+window;
            xmin = STATE_ESTIMATES(1,i)-window;
            ymax = STATE_ESTIMATES(2,i)+window;
            ymin = STATE_ESTIMATES(2,i)-window;
            zmax = STATE_ESTIMATES(3,i)+window;
            zmin = STATE_ESTIMATES(3,i)-window;
            
            xlim(subplot1,[xmin ,xmax])
            ylim(subplot1,[ymin,ymax])
            zlim(subplot1,[zmin,zmax])
         
            view_axis = [1,1,1]; % vector pointing from origin to camera
            view(subplot1,view_axis); % set to isometric view
            set(gca, 'YDir','reverse')
            set(gca, 'ZDir','reverse')
            

            plot3(STATE_ESTIMATES(1,1:i),STATE_ESTIMATES(2,1:i),STATE_ESTIMATES(3,1:i),'.-k','Parent',subplot1) % plots the state estimate
            hold on
            

            %ploting covariance elipse only every 20th time step 
            if rem(i,40) == 0
                plot3(xy_r_ellipse(:,1,i),xy_r_ellipse(:,2,i),ones(1,length(xy_r_ellipse(:,2)))*STATE_ESTIMATES(3,i),'-r','Parent',subplot1) % plots the covariance ellipse on the x,y plane
                hold on
                plot3(ones(1,length(yz_r_ellipse(:,2,i)))*STATE_ESTIMATES(1,i),yz_r_ellipse(:,1,i),yz_r_ellipse(:,2,i),'-r','Parent',subplot1) % plots the covariance ellipse on the y,z plane
                hold on 
                plot3(xz_r_ellipse(:,1,i),ones(1,length(xz_r_ellipse(:,2,i)))*STATE_ESTIMATES(2,i),xz_r_ellipse(:,2,i),'-r','Parent',subplot1) % plots the covariance ellipse on the x,z plane
                hold on
            end

             xlabel('x position (m)')
             ylabel('y position (m)')
             zlabel('z position (m)')
             title('Real Time State Estimate with Covariance Ellipsiod')

            
         subplot2 = subplot(2,3,[1,3]);
             % note that these axis limits created below do not create a cube,
             % so the relationship between x,y and z are not spacially
             % equivalent. However, when a cube is chosen, you can not see the
             % state estimate clearly (as the gps measurements are in the way
             xmax1 = max(STATE_ESTIMATES(1,:))+.1;
             xmin1 = min(STATE_ESTIMATES(1,:))-.1;
             ymax1 = max(STATE_ESTIMATES(2,:))+.1;
             ymin1 = min(STATE_ESTIMATES(2,:))-.1;
             zmax1 = max(STATE_ESTIMATES(3,:))+.1;
             zmin1 = min(STATE_ESTIMATES(3,:))-.1;

             % if you want a spacially accurate plotting window, use this:
%              xmax1 = 30;
%              xmin1 = -30;
%              ymax1 = 10;
%              ymin1 = -50;
%              zmax1 = 45;
%              zmin1 = -15;
             xlim(subplot2,[xmin1 ,xmax1])
             ylim(subplot2,[ymin1,ymax1])
             zlim(subplot2,[zmin1,zmax1])
             set(gca, 'YDir','reverse')
             set(gca, 'ZDir','reverse')
             view_axis = [-1,.1,3]; % vector pointing from origin to camera
             view(subplot2,view_axis); % set to isometric view

             plot3(STATE_ESTIMATES(1,:),STATE_ESTIMATES(2,:),STATE_ESTIMATES(3,:),'.-k','Parent',subplot2) % plot state estimate positions for all time
             hold on
             plot3(STATE_ESTIMATES(1,i),STATE_ESTIMATES(2,i),STATE_ESTIMATES(3,i),'-b','Parent',subplot2) % highlight current state estimate
             hold on
           if rem(i,40) == 0
                plot3(xy_r_ellipse(:,1,i),xy_r_ellipse(:,2,i),ones(1,length(xy_r_ellipse(:,2)))*STATE_ESTIMATES(3,i),'-r','Parent',subplot1) % plots the covariance ellipse on the x,y plane
                hold on
                plot3(ones(1,length(yz_r_ellipse(:,2,i)))*STATE_ESTIMATES(1,i),yz_r_ellipse(:,1,i),yz_r_ellipse(:,2,i),'-r','Parent',subplot1) % plots the covariance ellipse on the y,z plane
                hold on 
                plot3(xz_r_ellipse(:,1,i),ones(1,length(xz_r_ellipse(:,2,i)))*STATE_ESTIMATES(2,i),xz_r_ellipse(:,2,i),'-r','Parent',subplot1) % plots the covariance ellipse on the x,z plane
                hold on
            end
             
             xlabel('x position (m)')
             ylabel('y position (m)')
             zlabel('z position (m)')
             title('Full Path State Estimate Position')
             legend('Location','northeast')
             legend('Full State Estimate Path','Previous State Estiates','90% confidence Ellipsoid')


         pause(.001) % pause for real time path
         %hold off
    end
    
%%%%%%%%%%%%%% Rectangular Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%
%%

%% Static 3d Plot

%rectangle 
x_true = [0 0 1 1 0 0 12*.0254];
y_true = [0 0 0 1.5 1.5 0 0];
z_true = [0 -32 -32 -32 -32 -32 0]*0.0254;

figure(245)
plot3(x_true,-y_true,-z_true,'b--') % plots the state estimate
    hold on
plot3(STATE_ESTIMATES(1,:),-STATE_ESTIMATES(2,:),-STATE_ESTIMATES(3,:),'o')
axis([-0.75, 2.5, -1.8, .2, -1, 1])
xlabel('x position (m)')
ylabel('~y position (m)')
zlabel('~z position (m)')

%%

%% Motion plot with covariance ellipsoids Complex

%Calculate all covariance matrices before hand to improve plotting time
xy_r_ellipse = zeros(100,2,length(time));
yz_r_ellipse = zeros(100,2,length(time));
xz_r_ellipse = zeros(100,2,length(time));

for i = 1:length(time)
    if rem(i,40) == 0
                xy_r_ellipse(:,:,i) = cov_ellipse(STATE_ESTIMATES(1:2,i),SIGMA(1:2,1:2,i)); % covariance ellipse on xy plane
                yz_r_ellipse(:,:,i) = cov_ellipse(STATE_ESTIMATES(2:3,i),SIGMA(2:3,2:3,i)); % covariance ellipse on yz plane
                state_xz = [STATE_ESTIMATES(1,i),STATE_ESTIMATES(3,i)]; % state vector for X = [x:z]
                sig_xz = [SIGMA(1,1,i),SIGMA(1,3,i);SIGMA(3,1,i),SIGMA(3,3,i)]; % covariance matrix for X = [x;z]
                xz_r_ellipse(:,:,i) = cov_ellipse(state_xz,sig_xz); % covariance ellipse on xz plane
    end
end

figure(1)
plot3(STATE_ESTIMATES(1,:),STATE_ESTIMATES(2,:),STATE_ESTIMATES(3,:),'--g') % plot state estimate positions for all time
hold on             
xmax1 = max(STATE_ESTIMATES(1,:))+.1;
xmin1 = min(STATE_ESTIMATES(1,:))-.1;
ymax1 = max(STATE_ESTIMATES(2,:))+.1;
ymin1 = min(STATE_ESTIMATES(2,:))-.1;
zmax1 = max(STATE_ESTIMATES(3,:))+.1;
zmin1 = min(STATE_ESTIMATES(3,:))-.1;
xlim([xmin1 ,xmax1])
ylim([ymin1,ymax1])
zlim([zmin1,zmax1])
set(gca, 'YDir','reverse')
set(gca, 'ZDir','reverse')
view_axis = [-1,.1,3]; % vector pointing from origin to camera
view(view_axis); % set to isometric view
            

for i = 1:length(time)

             plot3(STATE_ESTIMATES(1,i),STATE_ESTIMATES(2,i),STATE_ESTIMATES(3,i),'.-b') % highlight current state estimate
             hold on
           if rem(i,40) == 0
                i
                plot3(xy_r_ellipse(:,1,i),xy_r_ellipse(:,2,i),ones(1,length(xy_r_ellipse(:,2)))*STATE_ESTIMATES(3,i),'-r') % plots the covariance ellipse on the x,y plane
                hold on
                plot3(ones(1,length(yz_r_ellipse(:,2,i)))*STATE_ESTIMATES(1,i),yz_r_ellipse(:,1,i),yz_r_ellipse(:,2,i),'-r') % plots the covariance ellipse on the y,z plane
                hold on 
                plot3(xz_r_ellipse(:,1,i),ones(1,length(xz_r_ellipse(:,2,i)))*STATE_ESTIMATES(2,i),xz_r_ellipse(:,2,i),'-r') % plots the covariance ellipse on the x,z plane
                hold on
            end
             
             xlabel('x position (m)')
             ylabel('y position (m)')
             zlabel('z position (m)')
             title('Full Path State Estimate Position')
             legend('Location','northeast')
             legend('Full State Estimate Path','Previous State Estiates','90% confidence Ellipsoid')


         pause(.001) % pause for real time path
         %hold off
    end
    
%% Error analysis

%interpolate the truth data positions

x_true_path = [];
y_true_path = [];
z_true_path = [];

for i = 1:length(x_true)-1
    x_true_path = [x_true_path,linspace(x_true(i),x_true(i+1),100)];
    y_true_path = [y_true_path,linspace(y_true(i),y_true(i+1),100)];
    z_true_path = [z_true_path,linspace(z_true(i),z_true(i+1),100)];
end

%plot interpolated path

% figure(34)
% plot3(x_true_path,y_true_path,z_true_path)

% calculate RMS error for all points
RMSE = zeros(length(STATE_ESTIMATES(1,:)),1);
for i = 1:length(STATE_ESTIMATES(1,:))
    RMSEi = [];
    for j =1:length(x_true_path)
        RMSEi = [RMSEi,norm(STATE_ESTIMATES(1:3,i) - [x_true_path(j);y_true_path(j);z_true_path(j)])];
    end
        RMSE(i) = min(RMSEi);
end
index_stop = 2351
figure(1)

plot(time(1:index_stop),RMSE(1:index_stop))
title('Position Error Over Time')
xlabel('Time (s)')
ylabel('Error (m)')

actual_RMSE = sqrt(sum(RMSE(1:index_stop).^2/length(RMSE(1:index_stop))))

    



%%%%%%%%%%%%%% Large linear plots . %%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%
%large linear
x_true = [0 0 12];
y_true = [0 0 -3.5*12*.0254];
z_true = [0 -32 -32]*0.0254;

%% 2D covariance in xy plot
figure(1)
    plot(STATE_ESTIMATES(1,1:end),STATE_ESTIMATES(2,1:end),'b')
    hold on
    plot(x_true,y_true,'--')
    hold on
    xlim([-1 13])
    ylim([-7 7])
for i =1:length(STATE_ESTIMATES(1,1:end))
    if rem(i,10) == 0
                xy_r_ellipse = cov_ellipse(STATE_ESTIMATES(1:2,i),SIGMA(1:2,1:2,i)); % covariance ellipse on xy plane
                plot(xy_r_ellipse(:,1),xy_r_ellipse(:,2),'-r') % plots the covariance ellipse on the x,y plane
                hold on
    end
end
    legend('State Estimate','Truth Data','90% Confidence Ellipse')
    title('Top Down View of State Estimate')
    xlabel('x Position (m)')
    ylabel('y Position (m)')
    
%%    
%interpolate the truth data positions

x_true_path = [];
y_true_path = [];
z_true_path = [];

for i = 1:length(x_true)-1
    x_true_path = [x_true_path,linspace(x_true(i),x_true(i+1),100)];
    y_true_path = [y_true_path,linspace(y_true(i),y_true(i+1),100)];
    z_true_path = [z_true_path,linspace(z_true(i),z_true(i+1),100)];
end

%plot interpolated path

% figure(34)
% plot3(x_true_path,y_true_path,z_true_path)

% calculate RMS error for all points
RMSE = zeros(length(STATE_ESTIMATES(1,:)),1);
for i = 1:length(STATE_ESTIMATES(1,:))
    RMSEi = [];
    for j =1:length(x_true_path)
        RMSEi = [RMSEi,norm(STATE_ESTIMATES(1:3,i) - [x_true_path(j);y_true_path(j);z_true_path(j)])];
    end
        RMSE(i) = min(RMSEi);
end


index_stop = 600
figure(1)

plot(time(1:index_stop),RMSE(1:index_stop))
title('Position Error Over Time')
xlabel('Time (s)')
ylabel('Error (m)')

actual_RMSE = sqrt(sum(RMSE(1:index_stop).^2/length(RMSE(1:index_stop))))

%%%%%%%%%%%%%%% Complex2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
%large linear
x_true = [0 0 0 3.8 10 5 0 0 0];
y_true = [0 0 5 8.25 5 5-3.2 5 0 0];
z_true = [0 -32 -32 -32 -32 -32 -32 -32 0]*0.0254;

%% 2D covariance in xy plot

% [land,blah] = find(time>96.5); Complex2
[land,blah] = find(time>81);


figure(1)
    plot(STATE_ESTIMATES(1,200:900),STATE_ESTIMATES(2,200:900),'b')
    hold on
%     plot(x_true,y_true,'--')
    hold on
%     xlim([-1 13])
%     ylim([-4 10])
%     xlim([-1 13])
%     ylim([-4 10])
% for i =1:length(STATE_ESTIMATES(1,1:end))
%     if rem(i,10) == 0
%                 xy_r_ellipse = cov_ellipse(STATE_ESTIMATES(1:2,i),SIGMA(1:2,1:2,i)); % covariance ellipse on xy plane
%                 plot(xy_r_ellipse(:,1),xy_r_ellipse(:,2),'-r') % plots the covariance ellipse on the x,y plane
%                 hold on
%     end
% end
    legend('State Estimate','Truth Data','90% Confidence Ellipse')
    title('Top Down View of State Estimate')
    xlabel('x Position (m)')
    ylabel('y Position (m)')
    
%%    
%interpolate the truth data positions

x_true_path = [];
y_true_path = [];
z_true_path = [];

for i = 1:length(x_true)-1
    x_true_path = [x_true_path,linspace(x_true(i),x_true(i+1),100)];
    y_true_path = [y_true_path,linspace(y_true(i),y_true(i+1),100)];
    z_true_path = [z_true_path,linspace(z_true(i),z_true(i+1),100)];
end

%plot interpolated path

% figure(34)
% plot3(x_true_path,y_true_path,z_true_path)

% calculate RMS error for all points
RMSE = zeros(length(STATE_ESTIMATES(1,:)),1);
for i = 1:length(STATE_ESTIMATES(1,:))
    RMSEi = [];
    for j =1:length(x_true_path)
        RMSEi = [RMSEi,norm(STATE_ESTIMATES(1:3,i) - [x_true_path(j);y_true_path(j);z_true_path(j)])];
    end
        RMSE(i) = min(RMSEi);
end


index_stop = 600
figure(1)

plot(time(1:index_stop),RMSE(1:index_stop))
title('Position Error Over Time')
xlabel('Time (s)')
ylabel('Error (m)')

actual_RMSE = sqrt(sum(RMSE(1:index_stop).^2/length(RMSE(1:index_stop))))

