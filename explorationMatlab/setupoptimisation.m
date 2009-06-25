%% setupoptimisation
%
% *Description:*  this sets up the variables for the optimisation of the robot
% and Exploration, this can be run by itself, nothing needs to be setup

%% Function Call
% 
% *Inputs:* NULL
%
% *Returns:* NULL

function setupoptimisation()

%% Variables
global optimise

%% Inverse kinematics:
%this is the min stream length
optimise.mintargetdis=0.15;
%this is the max stream length
optimise.maxtargetdis=0.60;
%this is the min distance between where stream hits target the target and
%where we desire it to hit (plane.home_point)
optimise.minAccepDis=0.015;
% This is how many max iterations for the optimisation
optimise.iLimit=25;%used to be 1000
% This is how many max iterations for the optimisation
optimise.funLimit=1000;%used to be 1000
% This is the acceptable value for the cost funciton minimisation
optimise.stol = 1e-4;%for PhD it is 1e-10;%used to be 1e-13...note: unless it is 1e-2 or more it basically always goes to the iteration limit
%the max angular deflection (in rads) from requested in pose selection
optimise.maxDeflectionError = pi/3;%used to be pi/8;
%the min angular deflection (in rads) from requested in pose selection
optimise.minDeflectionError = pi/180;%really shouldn't be 0 deg
% this takes 10 ms everytime it is called and it need only be called once
optimise.options = optimset('Display', 'off', 'Largescale', 'off', 'TolFun', optimise.stol,'MaxIter', optimise.iLimit,'MaxFunEvals',optimise.funLimit);
% optimise.options = optimset('jacobian', 'off','Display', 'off', 'Largescale', 'off', 'TolFun', optimise.stol,'MaxFunEvals', optimise.iLimit);
% optimise.options = optimset('Display', 'off', 'Largescale', 'off',
% 'TolFun', optimise.stol,'MaxFunEvals', optimise.iLimit,'DiffMinChange',0.01*pi/180);


%% Path plannning
%How many times we try and find a path before stopping
optimise.numofPPiterations=50;
%max angle (rads) for the joints 1,2,3 so we work out incremental steps
%with no collisions. Initially this was max_angle_for123=[9,11,20]*pi/180; 
%As worked out with cos formul to keep max posible movement at end effector
%to be approx 150mm and about the same size as safety dist in end ellipse 
optimise.max_angle_for123=[4,5,7]*pi/180; %Less than 1/2 calculated for safety
%the max angle that joints 4->6 can move without checks (calculated
%heuristically)
optimise.maxangleJ4to6=20*pi/180;

% watet based path planning this is how cource the graph search should be
% where 1 would be equal to the max_angle_for123
optimise.waterPPleaniancy=1.8;%2.5;

%% NBV
%this is how many views to save for the NBV
optimise.num_bestviews=60;
%This is how many paths to get for NBV
optimise.valid_max=20; 
% This is weighting for the jointmoveweight - which we want to minimise 
optimise.jointmoveweight=1;
% and the additional info which we want to maximise
optimise.addinfoweight=100;  

%% NBV_beta
%this will be multiplied by the percentage of the points that we know in
%space around the robot. It we know nothing then most possibliities will
%pass through but there is less change that the robot arm will be allowed
%at that spot. As we know more and the tr(1:3,4) of the arm in nbv_beta
%results in safe places this dampner will stop all possibilites being
%explored and only some will be looked at (still random)
optimise.nbv_beta_dampner=0.8;

