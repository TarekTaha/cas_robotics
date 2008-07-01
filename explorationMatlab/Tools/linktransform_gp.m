%% linktransform_gp
% *Description: This replaces the L{link num}(angle) calculation since that
% is slow, this is about 5-10 times quicker but with no extra checks, only
% returns the transform
%

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

function t=linktransform_gp(a,b)

%% Variables
% get the variables out of what was passed
alpha = a(1);
an = a(2);
dn = a(3);

% Angle to rotate and current offset
theta = a(4)+b;

% predetermine sine and cos of alpha and theta
sa = sin(alpha); 
ca = cos(alpha);

st = sin(theta); 
ct = cos(theta);

% return the transform
t =    [ct	-st*ca	st*sa	an*ct
        st	ct*ca	-ct*sa	an*st
        0	sa	ca	dn
        0	0	0	1];