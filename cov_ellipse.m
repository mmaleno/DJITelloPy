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


chisquare_val = 1.605;
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