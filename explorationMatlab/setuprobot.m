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

function setuprobot(numjoints)

% whether we are using a blasting or a scanning robot
if nargin==0
    numjoints=6;
end

%% Variables
global Q r densoobj robot_maxreach;

%New robot - start pose
Q=[0,-75,160,0,30,0]*pi/180;
% Default poses - add additional ones as needed
default_Q=[Q;[0,-88*pi/180,140*pi/180,0,-15*pi/180,0]];
default_Q=[default_Q;[0,-88*pi/180,98*pi/180,0,-15*pi/180,0]];

% uiwait(msgbox('Changing the default Q - delete me later'));
display('Changing the default Q - delete me later');
Q=[0,-88*pi/180,140*pi/180,0,-15*pi/180,0];
default_Q=[Q;[0,-88*pi/180,98*pi/180,0,-15*pi/180,0]];



if numjoints==6
    robot_maxreach.default_Q=default_Q;
    ellipse_safetyfactor=0.03; %(0.1 is 10%) used to be 0.1 but now we are using stephens model for ploting its better
    display(['ellipse_safetyfactor is set to ',num2str(ellipse_safetyfactor)]);
else
    Q=[Q,0];
    robot_maxreach.default_Q=[default_Q,zeros([size(default_Q,1),1])];    
    ellipse_safetyfactor=0.01; %(0.01 is 1%)
    display(['ellipse_safetyfactor is set to ',num2str(ellipse_safetyfactor)]);
end
%adding safety factor as a vvariable that stays with us
robot_maxreach.ellipse_safetyfactor=ellipse_safetyfactor;


%this is the robot object you wish to use
% try r = feval('rob_object');
try r = densoVM6083(numjoints);
catch
    error('Cant setup robot object');
end


%% Load model, add laser, Calcellipses to go around each robot piece

%this loads the model used and calculates the elipses
% MAKE SURE DENSOOBJ.mat has a variable called densoobj !!
%load densoobj.mat
%use stephens verts to make the ellipsoids instead
display('Using stephens models now not toms');
L=r.link;
n=r.n;
for i=1:n
    densoobj(i).M=L{i}.glyph.vertices;
    densoobj(i).F=L{i}.glyph.faces;
end
%this is a hack, put the same peice on the end twice so that we can still
%ahve the same sized densoobj and still ignore the first peice in most of
%the code
if i==6
    densoobj(i+1).M=L{i}.glyph.vertices;
    densoobj(i+1).F=L{i}.glyph.faces;
end

for piece=1:size(densoobj,2)
    [densoobj(piece).ellipse.x,densoobj(piece).ellipse.y,densoobj(piece).ellipse.z,...
        densoobj(piece).ellipse.params, densoobj(piece).ellipse.center]=calc_elip(piece,densoobj(piece).M,ellipse_safetyfactor);
end



%% Speed up variables for NBV
%this is used to make the NBV quicker so it will only try realistic points
%we know that the max reach from the first joint (swivel) is max about 1.1
%meters so don't even try anything that is larger than that

robot_maxreach.val=1.25;

n = r.n;
L = r.link;
t = r.base;
t = t * L{1}(Q(1));
robot_maxreach.firstlinkpos=[0,t(2:3,4)'];

%% Tag onto this var the scan/move speed of robot (should put in densoobj)

robot_maxreach.scan_speed=5;
robot_maxreach.move_speed=70;

%% Used to hold all paths for the robot
robot_maxreach.path=[];
%these are the points that are inside the elispses inside all the paths,
%not per path
robot_maxreach.pointcarvedout=[];

%% For movement of the arm, the angular difference
robot_maxreach.realMovementAngleInc=5;

%% Min Joint resolution in rads
robot_maxreach.minjointres=0.02*pi/180;
