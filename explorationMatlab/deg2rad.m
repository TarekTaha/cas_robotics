%% deg2rad
%
% *Description:*  Changes D (degrees) passed in to R (rads)

%% Function Call
% 
% * *Inputs:* D (double) angle in degrees
% * *Returns:* R (double) radians

function R=deg2rad(D)

%% Calculate radian from degrees
% $$ R=D\times \frac{\pi}{180} $$
R=D*pi/180;