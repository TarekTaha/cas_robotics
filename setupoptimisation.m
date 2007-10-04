%% setupoptimisation
%
% *Description:*  this sets up the variables for the optimisation of the robot
% and Exploration, this can be run by itself, nothing needs to be setup

%% Function Call
% 
% * *Inputs:* Null
% * *Returns:* Null

function setupoptimisation()

%% Variables
global optimise

%% Inverse kinematics:
% This is the min acceptable distance to target
optimise.minAccepDis=0.15;
% This is how many max iterations for the optimisation
optimise.iLimit=150;%used to be 1000
% This is the acceptable value for the cost funciton minimisation
optimise.stol = 1e-1;%used to be 1e-13
% This is weighting for the jointmoveweight - which we want to minimise 
optimise.jointmoveweight=1;
% and the additional info which we want to maximise
optimise.addinfoweight=50;  
%the max angular deflection (in rads) from requested in pose selection
optimise.maxDeflectionError = pi/6;%used to be pi/8;
%distance      /|
%to target is /_| the height of the triangle with 1 unit hypotenuse
optimise.maxtargetdis=sin(optimise.maxDeflectionError*2);

%% Path plannning
%How many times we try and find a path before stopping
optimise.numofPPiterations=50;
%max angle (rads) for the joints 1,2,3 so we work out incremental steps
%with no collisions. Initially this was max_angle_for123=[9,11,20]*pi/180; 
%As worked out with cos formul to keep max posible movement at end effector
%to be approx 150mm and about the same size as safety dist in end ellipse 
optimise.max_angle_for123=[4,5,7]*pi/180; %Less than 1/2 calculated for safety

%% NBV
%this is how many views to save for the NBV
optimise.num_bestviews=60;
%This is how many paths to get for NBV
optimise.valid_max=20; 

%% NBV_beta
%this will be multiplied by the percentage of the points that we know in
%space around the robot. It we know nothing then most possibliities will
%pass through but there is less change that the robot arm will be allowed
%at that spot. As we know more and the tr(1:3,4) of the arm in nbv_beta
%results in safe places this dampner will stop all possibilites being
%explored and only some will be looked at (still random)
optimise.nbv_beta_dampner=0.8;
