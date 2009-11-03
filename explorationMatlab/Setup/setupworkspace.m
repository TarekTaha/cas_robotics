%% setupworkspace
%
% *Description:* This sets the variables for the workspace with their
% different weightings and it draws this up in the main figure the robot
% (r) must be previously setup

%% Function Call
% 
% *Inputs:* 
%
% _showunknownpnts_ (bin) whether or not to display unknown points
%
% *Returns:* NULL
function setupworkspace(showunknownpnts)

%% Variables
%Declare global variables
global workspace r Q;

guiParams=getappdata(gcf,'guiParams');

% Check Inputs: default is not to show the points
if nargin<1
    showunknownpnts=false;
end

%The voxel increment size
workspace.inc_size=0.09;

%The only point known is where the robot arm is
initialpoint=fkine(r,Q);
workspace.knowncoords=[initialpoint(1:3,4)]';
% make it fit to the grid
workspace.knowncoords=(round(workspace.knowncoords./workspace.inc_size))*workspace.inc_size;

%The min and max of the workspace
workspace.min=[-1.6,-1.6,-workspace.inc_size/2];
workspace.max=[1.6,1.6,2.5];
%make it fit to the grid
workspace.min=workspace.inc_size.*round(workspace.min/workspace.inc_size);
workspace.max=workspace.inc_size.*round(workspace.max/workspace.inc_size);

%holds the radius size for surfaces for exploration
workspace.mew=0.30;

%holds the robot basic unmovable size which takes points away from expecting scans
workspace.robotsize=[-0.45,0.45;
                     -0.45,0.45;
                        0,0.89];

%These are the home point of the disk planes that are at 0,0,0 and halfway
%up the robot: this stops the robot trying to self scan since it will not
%see anothing looking through itself, THESE HAVE ALSO BEEN INDEXED
workspace.robotplanes.home_points=round([zeros([3,3]);...
    ones([3,1])*[0,0,sum(workspace.robotsize(3,:))/2]]./workspace.inc_size)*workspace.inc_size;                               
%These are the equations of the planes
workspace.robotplanes.equ=[1,0,0,0; 0,1,0,0; 0,0,1,0;
                           1,0,0,0; 0,1,0,0; 0,0,1,sum(workspace.robotsize(3,:))/2];

%this will hold the obstacles in voxeles (using inc_size)
workspace.indexedobsticles=[];

%this makes nbv_volume quicker by indexing the home points
workspace.indexedobsticles_home_point=[];
workspace.indexedobsticles_equ=[];

% There are 4 bands 
% 1) robot workspace
% 2) max blast
% 3) blast danger
% 4) all else don't care about
%*level 1* is where the robot can move
workspace.impLev(1).x=[-1.3,1.4];
workspace.impLev(1).y=[-1.4,1.4];
workspace.impLev(1).z=[workspace.inc_size,1.7];
%*level 2* is where the robot can blast but not move
workspace.impLev(2).x=[-1.4,1.55];
workspace.impLev(2).y=[-1.5,1.5];
workspace.impLev(2).z=[0,2.1];
%*level 3* is where the robot can shoot sand but not blast and not move
workspace.impLev(3).x=[workspace.min(1),workspace.max(1)];
workspace.impLev(3).y=[workspace.min(2),workspace.max(2)];
workspace.impLev(3).z=[workspace.min(3),workspace.max(3)];

% This is the weight given to info in each of the levels
% *Note* points in level 1 are also in level 2 and 3 so they get weights for
% each (1), points in level 2 are also in 3 so they get both
%made them add to 1 in accordamce with PhD working
workspace.dotweight=[0.6,0.35,0.05];
%so as it would be smaller increments
workspace.unknowncoords=zeros([round(((workspace.max(1)-workspace.min(1))*...
                                      (workspace.max(2)-workspace.min(2))*...
                                      (workspace.max(3)-workspace.min(3)))/...
                                      workspace.inc_size^3),3]);
%% Adding floor obstacles for safety
points=[];
% __Old Method to add points from a point cloud
% uiwait(msgbox('inputting worksace from memory (pointsCloud.mat) and div by 1, then only using points within range, also index (inc_size) here is halved but this will change with gathering of more points'));
% load pointsCloud.mat;
% points=points(find(points(:,1)<.6 & points(:,1)>-.6),:);
% points=points(find(points(:,2)<1 & points(:,2)>-1),:);

% Add a floor layer
% uiwait(msgbox('Adding a floor layer only'));
tempx=[];tempy=[]; 
tempInc=workspace.inc_size;
for i=-.75:tempInc:.85
    tempx=[tempx;i*ones(length(-.65:tempInc:.75),1)];
    tempy=[tempy;(-.65:tempInc:.75)'];
end
tempz=workspace.impLev(1).z(1)*ones([size(tempx,1),1]);
points=[points;[tempx,tempy,tempz]];

%add the bottoms of the ibeams
tempx=-1.5:tempInc:1.5;
tempx=[tempx';tempx';tempx';tempx'];
tempy=-0.6*ones([size(tempx,1)/4,1]);
tempy=[tempy;-0.8*ones([size(tempx,1)/4,1])];
tempy=[tempy;0.5*ones([size(tempx,1)/4,1])];
tempy=[tempy;0.7*ones([size(tempx,1)/4,1])];
tempz=1*ones([size(tempx,1),1]);
points=[points;[tempx,tempy,tempz]];


% Only use points within the correct range
level1=GetImpLevInfo(points);
points=points(level1,:);
% workspace.indexedobsticles=unique(round(workspace.Nobsticlepoints/tempInc)*(tempInc),'rows');
workspace.indexedobsticles=unique(round(points/tempInc)*(tempInc),'rows');


% Could also load up freespace variable 
% a=load('workspace.mat');
% workspace.knowncoords=a.workspace.knowncoords;                                  

%% Setup the unknown workspace 
current_row=1;

temp_vals=round([workspace.min(3)-workspace.inc_size:workspace.inc_size:workspace.max(3)+workspace.inc_size]'./workspace.inc_size)*workspace.inc_size;
length_temp_vals=length(temp_vals);

for x=workspace.min(1)-workspace.inc_size:workspace.inc_size:workspace.max(1)+workspace.inc_size  
    for y=workspace.min(2)-workspace.inc_size:workspace.inc_size:workspace.max(2)+workspace.inc_size
        workspace.unknowncoords(current_row:current_row+length_temp_vals-1,:)=[x*ones([length_temp_vals,1]),y*ones([length_temp_vals,1]),temp_vals];
        current_row=current_row+length_temp_vals;
    end; 
end;

%round to the correct index
workspace.unknowncoords=round(workspace.unknowncoords/workspace.inc_size)*workspace.inc_size;

%find point that are not in the robotsize space
index=find((workspace.unknowncoords(:,1)<workspace.robotsize(1,2) & workspace.unknowncoords(:,1)>workspace.robotsize(1,1)) &...
           (workspace.unknowncoords(:,2)<workspace.robotsize(2,2) & workspace.unknowncoords(:,2)>workspace.robotsize(2,1)) &...
           (workspace.unknowncoords(:,3)<workspace.robotsize(3,2) & workspace.unknowncoords(:,3)>workspace.robotsize(3,1)));
%make all thoes points removed from UNKNOWN and put in KNOWN stack
workspace.knowncoords=[workspace.knowncoords;workspace.unknowncoords(index,:)];
% then change the values in unknown space to be only ouside of this
% newindex=setdiff([1:size(workspace.unknowncoords,1)]',index);
% workspace.unknowncoords=workspace.unknowncoords(newindex,:);


%make sure all points are within the workspace
[nothing1,nothing1,level3_un]=GetImpLevInfo(workspace.unknowncoords);
workspace.unknowncoords=workspace.unknowncoords(level3_un,:);
[workspace.lev1unknown,workspace.lev2unknown]=GetImpLevInfo(workspace.unknowncoords);

%% Plot the main unknown graph
hold on;

xlabel('x axis');ylabel('y axis');zlabel('z axis');
axis([workspace.min(1) workspace.max(1) workspace.min(2) workspace.max(2) workspace.min(3) workspace.max(3)]);
drawnow

%if we are going to show the different levels of importance
if showunknownpnts
    level1=GetImpLevInfo(workspace.unknowncoords);
%     _Old_ Gives the ability to plot all 3 levels of unknown
%     [level1,level2,level3]=GetImpLevInfo(workspace.unknowncoords);
%     level1=intersect(level2,level1);
%     level2=setdiff(level2,level1);
%     level3=setdiff(level3,union(level1,level2));
   
   % Plots for different weightings a different color
   guiParams.unknownplot=plot3(workspace.unknowncoords(level1,1),...
                               workspace.unknowncoords(level1,2),...
                               workspace.unknowncoords(level1,3),...
                               '.','Color',[1-workspace.dotweight(1) 1-workspace.dotweight(1) 1]);
    setappdata(gcf,'guiParams',guiParams);                             
%     _Old_ Gives the ability to plot all 3 levels of unknown
%    plot3(workspace.unknowncoords(level2,1),workspace.unknowncoords(level2,2),workspace.unknowncoords(level2,3),'.','Color',[1-workspace.dotweight(2) 1-workspace.dotweight(2) 1])
%    plot3(workspace.unknowncoords(level3,1),workspace.unknowncoords(level3,2),workspace.unknowncoords(level3,3),'.','Color',[1-workspace.dotweight(3) 1-workspace.dotweight(3) 1])
end

workspace.spec_pnts=[];



%% Setup the graph obstacles, 
% %this makes the check for a collision quicker
% obsticle_points=workspace.indexedobsticles(GetImpLevInfo(workspace.indexedobsticles),:);
% % If we include unknown
% indexed_knowncoords=putinVoxels_gp(setdiff(workspace.knowncoords(GetImpLevInfo(workspace.knowncoords),:),workspace.indexedobsticles,'rows'),workspace.inc_size);
% all_possible=putinVoxels_gp(workspace.unknowncoords(workspace.lev1unknown   ,:),workspace.inc_size);
% [nothing,index]=setdiff(all_possible,[indexed_knowncoords;obsticle_points],'rows');
% unknown_points=workspace.unknowncoords(workspace.lev1unknown(index),:);
% 
% update_jointconfig_obs(matsize,obsticle_points,unknown_points);
global graf_obs
try 
    load graf_obs.mat
catch
    uiwait(msgbox('couldnt load the graf_obs.mat file, make sure the planning directory is added to the maatlab path'));
end

%% Setup the classifier surface size
workspace.classifyMew=0.1;
workspace.classifyProbThreshhold=0.999;
% display('changed the classifyProbThreshhold threshold');
workspace.classifyConfidence=0.90;

%the classification cube size
workspace.class_cubesize=workspace.inc_size/4;

%variables for the classification threshholds
%minimum number of classifications in voxel to make it valid
workspace.minclassifications=30;
% minimmum majority classification to make it useful
workspace.classfierthreshhold=1.5;



