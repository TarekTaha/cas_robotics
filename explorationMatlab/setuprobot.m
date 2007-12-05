%% setuprobot
%
% *Description:* This sets up the robot, nothing needs to have been declared before this
% Sets up the robot (r), Joint positions Q, 
% and densoobj which is for force fielding, and plotting

%% Function Call
% 
% *Inputs:* NULL
%
% *Returns:* NULL

function setuprobot()

%% Variables
global Q r densoobj robot_maxreach;

%New robot - start pose
Q=[0,-75,160,0,30,0]*pi/180;
% Default poses - add additional ones as needed
default_Q=[Q;[0,-88*pi/180,140*pi/180,0,-15*pi/180,0]];
default_Q=[default_Q;[0,-88*pi/180,98*pi/180,0,-15*pi/180,0]];
robot_maxreach.default_Q=default_Q;

%this is the robot object you wish to use
try r = feval('rob_object');
catch
    error('Cant setup robot object');
end

%% Load model, add laser, Calcellipses to go around each robot piece

%this loads the model used and calculates the elipses
% MAKE SURE DENSOOBJ.mat has a variable called densoobj !!
load densoobj.mat
%CHANGE last peice so it sort of has the laser on it
densoobj(7).M(find(densoobj(7).M(:,3)>=-0.001 & densoobj(7).M(:,1)<0),3)=0.12;

for piece=1:size(densoobj,2)
    [densoobj(piece).ellipse.x,densoobj(piece).ellipse.y,densoobj(piece).ellipse.z,...
        densoobj(piece).ellipse.params, densoobj(piece).ellipse.center]=calc_elip(piece,densoobj(piece).M);
end

%% Speed up variables for NBV
%this is used to make the NBV quicker so it will only try realistic points
%we know that the max reach from the first joint (swivel) is max about 1.1
%meters so don't even try anything that is larger than that

robot_maxreach.val=1.15;

n = r.n;
L = r.link;
t = r.base;
t = t * L{1}(Q(1));
robot_maxreach.firstlinkpos=[0,t(2:3,4)'];

%% Tag onto this var the scan/move speed of robot (should put in densoobj)

robot_maxreach.scan_speed=5;
robot_maxreach.move_speed=60;

%% Used to hold all paths for the robot
robot_maxreach.path=[];
%these are the points that are inside the elispses inside all the paths,
%not per path
robot_maxreach.pointcarvedout=[];