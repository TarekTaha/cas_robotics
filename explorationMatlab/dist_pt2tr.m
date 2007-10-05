%% dist_pt2tr
%
% *Description:*  Function for find the distance from a point to a
% transform

%% Function Call
% 
% *Inputs:* 
%
% _pt_ (3*1 double) x,y,z cartesian point
%
% _tr_ (4*4 double) transform of an arm
%
% *Returns:* 
%
% _dist_ (double) distance from pt to tr

function dist=dist_pt2tr(pt,tr)

%% Given an end effector transform
% $$tr=\begin{array}{cccc} 
% n_x & o_x & a_x & P_x \\ 
% n_y & o_y & a_y & P_y \\
% n_z & o_z & a_z & P_z \\
% 0&0&0&1\end{array}$$

%% Calculate distance (dist) from pt to tr
% $$dist=\sqrt{(pt_x-P_x)^2+(pt_y-P_y)^2 + (pt_z-P_z)^2}$$
dist=sqrt((pt(1)-tr(1,4))^2+...
          (pt(2)-tr(2,4))^2+...
          (pt(3)-tr(3,4))^2);
