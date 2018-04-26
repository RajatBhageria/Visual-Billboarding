function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
% x is the video points and x' is the logo_points

xs = video_pts(:,1);
ys = video_pts(:,2); 
xps = logo_pts(:,1);
yps = logo_pts(:,2); 

ax = [-xs, -ys, -ones(4,1), zeros(4,1),zeros(4,1),zeros(4,1), xs.*xps, ys.*xps, xps];
ay = [zeros(4,1), zeros(4,1), zeros(4,1), -xs, -ys, -ones(4,1), xs.*yps, ys.*yps, yps];
A = [ax; ay];

%find solution to Ah =e 
[~,~,V] = svd(A); 

%find the h vector 
h = V(:,end);

%reshape vector 
H = reshape(h,[3,3]);
H = H'; 
H = H / V(9,9);

end

