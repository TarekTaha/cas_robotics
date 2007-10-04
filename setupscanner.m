%% setupscanner
%
% *Description:* This function sets up the variables of the scanner
% NOTE: The robot must be defined at which position it is
% AND the workspace must be defined since this is 

%% Function Call
% 
% *Inputs:* Null
% *Returns:* Null

function setupscanner()

%% Variables
global scan r Q

%the first scan origin is where the robot arm is
initialpoint=fkine(r,Q);
scan.origin=[initialpoint(1:3,4)]';

%sets up the chosen view vector
scan.chosenview=unit([1,1,1]);

%max range of laser - made smaller so estimate is smaller
scan.size=1.6; 

% How many times we have scanned
scan.tries=0;

% The origin of all scans so far
scan.ALLorigin=[];


%Laser Angualar VARIABLES
%this is the angle either side of the bearing of the center of the scan \|/
%scan.theta=pi/6;
scan.theta=120*pi/180; %used to be pi/2;

%this is the incremental steps
scan.theta_incr=0.36*(pi/180);
%this is the angle from the tilt, - is up, + is down, angle must be from -2pi to 2pi
scan.alpha=95*pi/180;
%if joint 3 is greater than this limiting condition then alpha changes
scan.alpha_limited_condition=150*pi/180;
scan.alpha_limited=30*pi/180;

%this is for the actual robot and how many points we can leave off so we
%want how many points per cube at the max range?
% so we don't use have to use all data to 
scan.numpntsInCube=1; % this can be anything greater than 0, a value of 0.5 would mean 1 point in every 2 cubes at the max scan value

%this is how many tilts we actually want to use a ratio of 1:this num
%THIS IS NOT INCLUDED CURRENTLY, NEED TO DO WORK TO INCLUDE IT
display('grep to see if this variable is used since I just commented it out');
keyboard
% scan.tiltstouse=2;

% we don't want to have to do the scan projection everytime so we will load
% it, however if any of the parameters (alpha theta) change then it will be
% incorrect. Also variables from workspace global var are used too
try load('basic_scan_data.mat');
    % scan.basescan=-1*basic_scan_data;
    scan.basescan=basescan;
catch; display('could not find the file basic_scan_data.mat');
end    

%this is the best views that we have already gone to, this will be in rads
%and is used in nbv_beta since this has predefined poses
scan.done_bestviews_orfailed=[inf, inf, inf, inf, inf, inf];
