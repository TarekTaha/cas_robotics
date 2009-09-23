%% dist2pts
%
% *Description:*  Function for find the distance between 2 or the same number of 3D points

%% Function Call
% 
% *Inputs:* 
%
% _pt1_ (many*3 double) x,y,z cartesian point
%
% _pt2_ (many*3 double) x,y,z cartesian point
%
% *Returns:* 
%
% _dist_ (double) distance from pt1 to pt2

function dist=dist2pts(pt1,pt2)

%% Calculate distance (dist) from pt to tr
% $$dist=\sqrt{(pt1_x-pt2_x)^2+(p1t_y-p2t_y)^2 + (pt1_z-pt2_z)^2}$$
dist=sqrt((pt1(:,1)-pt2(:,1))^2+...
          (pt1(:,2)-pt2(:,2))^2+...
          (pt1(:,3)-pt2(:,3))^2);
