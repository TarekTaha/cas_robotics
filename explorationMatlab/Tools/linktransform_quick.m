%% linktransform_quick
function t=linktransform_quick(an,dn,theta,sa,ca)
%% Function Call
%
% *Inputs:* 
%
% _a_ (4*1 double) [L{linknun}.alpha,L{linknun}.a,L{linknun}.d,L{linknun}.offset] from robot
%
% _b_ (double) angle in radians to be rotated
%
% *Returns:* 
%
% _t_ (4*4 double) homogeneous transform



st = sin(theta); 
ct = cos(theta);

% return the transform
t =    [ct	-st*ca	st*sa	an*ct
        st	ct*ca	-ct*sa	an*st
        0	sa	ca	dn
        0	0	0	1];
end