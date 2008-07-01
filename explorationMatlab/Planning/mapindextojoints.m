%% mapindextojoints
%
% *Description*: This function maps discrete points to config space joint anglespoints to 

%% Function Call
%
% *Inputs:* a (size?, type?) index joint 1 
%
% _b_ (size?, type?) index joint 2
%
% _c_ (size?, type?) index joint 3
%
% _qlimits_ (size?, type?) :Limits of robot r.qlim
%
% _matsize_ (size?, type?) :3dimensional num of discrete points in config space in each direction
%
% *Outputs:* J1 (size?, type?) :Joint 1 number from 1-matsize(1)
%
% _J2_ (size?, type?) :Joint 2 num from 1-matsize(2)
%
% _J3_ (size?, type?) :Joint 3 num 1-matsize(3)
%

function [J1,J2,J3]=mapindextojoints(a,b,c,qlimits,matsize)

%% Variables
nelements=length(a);
J1=zeros([nelements,1]);J2=J1;J3=J1;

%% Functions
for i=1:nelements
    temp=[a(i);b(i);c(i)].*(qlimits(1:3,2)-qlimits(1:3,1))./matsize+qlimits(1:3,1);
    J1(i)=temp(1);J2(i)=temp(2);J3(i)=temp(3);
end