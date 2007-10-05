%% deg2rad
%
% *Description:*  Changes D (degrees) passed in to R (rads)

%% Function Call
% 
% *Inputs:* 
%
% _D_ (double) angle in degrees
%
% *Returns:* 
%
% _R_ (double) radians

function R=deg2rad(D)

%% Calculate radian from degrees
% $$ R=D\times \frac{\pi}{180} $$
R=D*pi/180;