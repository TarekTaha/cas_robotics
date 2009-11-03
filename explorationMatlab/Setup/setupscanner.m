%% setupscanner
%
% *Description:* This function sets up the variables of the scanner
% NOTE: The robot must be defined at which position it is
% AND the workspace must be defined since this is 

%% Function Call
% 
% *Inputs:* NULL
%
% *Returns:* NULL

function setupscanner()

%% Variables
global G_scan

%sets up the chosen view vector
G_scan.chosenview=unit([1,1,1]);

%max range of laser - made smaller so estimate is smaller
G_scan.range=1.6; 

% How many times we have scanned
G_scan.tries=0;

%the maximum exporation steps
G_scan.maxtries=8;


% The origin of all scans so far
G_scan.ALLorigin=[];

%Laser Angualar VARIABLES
%this is the incremental steps
G_scan.theta_incr=0.36*(pi/180);
%this is the angle from the tilt, - is up, + is down, angle must be from -2pi to 2pi
G_scan.alpha=95*pi/180;
%if joint 3 is greater than this limiting condition then alpha changes
G_scan.alpha_limited_condition=150*pi/180;
G_scan.alpha_limited=30*pi/180;

%this is for the actual robot and how many points we can leave off so we
%want how many points per cube at the max range?
% so we don't use have to use all data to 
G_scan.numpntsInCube=1; % this can be anything greater than 0, a value of 0.5 would mean 1 point in every 2 cubes at the max G_scan value

% we don't want to have to do the G_scan projection everytime so we will load
% it, however if any of the parameters (alpha theta) change then it will be
% incorrect. Also variables from workspace global var are used too
% uiwait(msgbox('Not loading basic_scan exploration data'));
try load('basic_scan_data.mat');
    G_scan.basescan=basescan;
catch %#ok<CTCH>
    display('could not find the file basic_scan_data.mat');
end    

%this is the best views that we have already gone to, this will be in rads
%and is used in nbv_beta since this has predefined poses
G_scan.done_bestviews_orfailed=[inf, inf, inf, inf, inf, inf];

% uiwait(msgbox('Not loading all_views exploration data'));
% try load all_views.mat
% catch
%     display('Cant call or load all_views.mat');
% end
