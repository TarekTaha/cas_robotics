%% rad2deg
%
% *Description:*  Changes R (rads) passed in to D (degrees)

%% Function Call
% 
% *Inputs:* 
%
% _R_ (double) radians
%
% *Returns:* 
%
% _D_ (double) angle in degrees

function D=rad2deg(R)

%% Calculate radian from degrees
% $$ D=R\times \frac{180}{\pi} $$
D=R*180/pi;