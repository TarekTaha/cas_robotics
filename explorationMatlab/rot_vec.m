%% FUNCTION: rot_vec
%this function rotates a vector around another one a certain num RADIANS

function result=rot_vec(vec,rotvec,theta)
%make into unit vector
u=rotvec(:)/norm(rotvec);
 
cosa = cos(theta);
sina = sin(theta);
vera = 1 - cosa;
x = u(1);
y = u(2);
z = u(3);
rot = [cosa+x^2*vera x*y*vera-z*sina x*z*vera+y*sina; ...
       x*y*vera+z*sina cosa+y^2*vera y*z*vera-x*sina; ...
       x*z*vera-y*sina y*z*vera+x*sina cosa+z^2*vera]'; 

result=vec*rot;
