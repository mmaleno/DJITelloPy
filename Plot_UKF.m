%%%%%%%%%%%%%%%%% Plotting %%%%%%%%%%%%%%%%%%%%%
% This script is used to plot the output of run_UKF.m
% Authors: Kevin Shoyer and Max Maleno
% Date: 5/1/2020

%% 3D Motion Plot
% true data
% 1 in = 0.0254m
x_true = [0 0 2/.0254 0 3]* 0.0254;
y_true = [0 0 0 0 0];
z_true = [0 -32 -32 -32 0]*0.0254;

figure(24) %3D plot

persistance = 25;
delay = 0.05;
for i = 1:length(STATE_ESTIMATES(1,:))
    plot3(x_true,-y_true,-z_true,'b--') % plots the state estimate
    hold on
    if i<persistance+1
        plot3(STATE_ESTIMATES(1,1:i),-STATE_ESTIMATES(2,1:i),-STATE_ESTIMATES(3,1:i),'o')
    else
        plot3(STATE_ESTIMATES(1,i-persistance:i),-STATE_ESTIMATES(2,i-persistance:i), ...
                                            -STATE_ESTIMATES(3,i-persistance:i),'o')
    end
    xlabel('x position (m)')
    ylabel('~y position (m)')
    zlabel('~z position (m)')
    axis([-0.75, 2.5, -0.2, 0.4, -1, 1])
    pause(delay)
    hold off
end
%% Static 3d Plot
% linear
% x_true = [0 0 2/.0254 0 3]* 0.0254;
% y_true = [0 0 0 0 0];
% z_true = [0 -32 -32 -32 0]*0.0254;

%rectangle 
x_true = [0 0 1 1 0 0 12*.0254];
y_true = [0 0 0 1.5 1.5 0 0];
z_true = [0 -32 -32 -32 -32 -32 0]*0.0254;

figure(245)
plot3(x_true,-y_true,-z_true,'b--') % plots the state estimate
    hold on
plot3(STATE_ESTIMATES(1,:),-STATE_ESTIMATES(2,:),-STATE_ESTIMATES(3,:),'o')
axis([-0.75, 2.5, -0.2, 0.4, -1, 1])
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
for i =1:length(STATE_ESTIMATES(1,:))
    if rem(i,40) == 0
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
                plot3(STATE_ESTIMATES(1,:),STATE_ESTIMATES(2,:),STATE_ESTIMATES(3,:),'--')
                hold on
                plot3(x_true,y_true,z_true)
    end
end

%% 2D covariance in xy plot
figure(1)
for i =1:length(STATE_ESTIMATES(1,:))
    if rem(i,30) == 0
                xy_r_ellipse = cov_ellipse(STATE_ESTIMATES(1:2,i),SIGMA(1:2,1:2,i)); % covariance ellipse on xy plane
                plot(xy_r_ellipse(:,1),xy_r_ellipse(:,2),'-r') % plots the covariance ellipse on the x,y plane
                hold on
                plot(STATE_ESTIMATES(1,:),STATE_ESTIMATES(2,:))
    end
end

%% GARBAGE Motion plot with covariance ellipses
figure(1)
    window = .5; % used to update moving window
    for i = 1:length(time)

        %live plot of the covariance ellipsiod and the state estimate 
        subplot1 = subplot(2,4,4);

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

            plot3(STATE_ESTIMATES(1,1:i),STATE_ESTIMATES(2,1:i),STATE_ESTIMATES(3,1:i),'.-k','Parent',subplot1) % plots the state estimate
            hold on
            

            %ploting covariance elipse only every 20th time step 
            if rem(i,20) == 0
                xy_r_ellipse = cov_ellipse(STATE_ESTIMATES(1:2,i),SIGMA(1:2,1:2,i)); % covariance ellipse on xy plane
                yz_r_ellipse = cov_ellipse(STATE_ESTIMATES(2:3,i),SIGMA(2:3,2:3,i)); % covariance ellipse on yz plane
                state_xz = [STATE_ESTIMATES(1,i),STATE_ESTIMATES(3,i)]; % state vector for X = [x:z]
                sig_xz = [SIGMA(1,1,i),SIGMA(1,3,i);SIGMA(3,1,i),SIGMA(3,3,i)]; % covariance matrix for X = [x;z]
                xz_r_ellipse = cov_ellipse(state_xz,sig_xz); % covariance ellipse on xz plane
                plot3(xy_r_ellipse(:,1),xy_r_ellipse(:,2),ones(1,length(xy_r_ellipse(:,2)))*STATE_ESTIMATES(3,i),'-r','Parent',subplot1) % plots the covariance ellipse on the x,y plane
                hold on
                plot3(ones(1,length(yz_r_ellipse(:,2)))*STATE_ESTIMATES(1,i),yz_r_ellipse(:,1),yz_r_ellipse(:,2),'-r','Parent',subplot1) % plots the covariance ellipse on the y,z plane
                hold on 
                plot3(xz_r_ellipse(:,1),ones(1,length(xz_r_ellipse(:,2)))*STATE_ESTIMATES(2,i),xz_r_ellipse(:,2),'-r','Parent',subplot1) % plots the covariance ellipse on the x,z plane
                hold on
            end

             xlabel('x position (m)')
             ylabel('y position (m)')
             zlabel('z position (m)')
             title('Real Time State Estimate with Covariance Ellipsiod')


         subplot2 = subplot(2,4,[1,3]);
             % note that these axis limits created below do not create a cube,
             % so the relationship between x,y and z are not spacially
             % equivalent. However, when a cube is chosen, you can not see the
             % state estimate clearly (as the gps measurements are in the way
             xmax1 = max(STATE_ESTIMATES(1,:))+1;
             xmin1 = min(STATE_ESTIMATES(1,:))-1;
             ymax1 = max(STATE_ESTIMATES(2,:))+1;
             ymin1 = min(STATE_ESTIMATES(2,:))-1;
             zmax1 = max(STATE_ESTIMATES(3,:))+1;
             zmin1 = min(STATE_ESTIMATES(3,:))-1;

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
             view_axis = [1,1,1]; % vector pointing from origin to camera
             view(subplot2,view_axis); % set to isometric view

             plot3(STATE_ESTIMATES(1,:),STATE_ESTIMATES(2,:),STATE_ESTIMATES(3,:),'.-k','Parent',subplot2) % plot state estimate positions for all time
             hold on
             plot3(STATE_ESTIMATES(1,i),STATE_ESTIMATES(2,i),STATE_ESTIMATES(3,i),'or','Parent',subplot2) % highlight current state estimate
             hold on
             

             xlabel('x position (m)')
             ylabel('y position (m)')
             zlabel('z position (m)')
             title('Full Path State Estimate Position')
             legend('Location','northeast')
             legend('Full State Estimate Path','Previous State Estiates','Raw GPS Measurements')

         subplot3 = subplot(2,4,[5,7]);
             yaw_std = squeeze(sqrt(SIGMA(9,9,:))); % squeeze gets rid of extra dimensions, sqrt because we want to plot standard deviation
             errorbar(time,STATE_ESTIMATES(9,:),yaw_std,'.k')
             hold on
    %          plot(time,yaw,'xb') % too messy to plot raw data too

             xlabel('Time (s)')
             ylabel('Angle (rad)')
             title('Yaw State Estimate with Standard Deviation Error Bounds')

          subplot4 = subplot(2,4,8);
                     %plot(time,STATE_ESTIMATES(9,:),'.k')
             hold on
             yaw_std = squeeze(sqrt(SIGMA(9,9,:))); % squeeze gets rid of extra dimensions, sqrt because we want to plot standard deviation
             errorbar(time(1:20),STATE_ESTIMATES(9,1:20),yaw_std(1:20),'.k')
             hold on
    %          plot(time,yaw,'xb') % too messy to plot raw data too

             xlabel('Time (s)')
             ylabel('Angle (rad)')
             title(' Zoomed in Yaw State Estimate')


         pause(.01) % pause for real time path
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

