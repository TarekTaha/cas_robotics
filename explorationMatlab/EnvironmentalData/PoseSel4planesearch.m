%Note: This assumes we wish to blast in the direction where the normal
%is pointing towards the origin which is the base center (0,0,0)
        
function [pose]=PoseSel4planesearch(plane,displayon)
global r Q optimise

if nargin<2
    displayon=true;
end

% Turn off warning messages if we dont care
if ~displayon
    warning off
end

%% Setup the pose sel object
try global robmap_h; 
    if isempty(robmap_h) 
        try robmap_h.release;pause(1);end;
        robmap_h=actxserver('EyeInHand.SurfaceMap');
        robmap_h.registerevent(@myhandler);
    end
    %get the Denso Blasting Cost Function
    try global DensoBlasting_h
        if isempty(DensoBlasting_h)
            DensoBlasting_h = robmap_h.GetDensoBlastingCostFunction;
        end
    catch
        display('EyeInHand Problem: Unable to create DensoBlastingCost from surface map')
    end
catch
    display('EyeInHand Problem: Unable to create surface map')
end

%% Variables 
numlinks = r.n;
Links = r.link;
qlimits=r.qlim; 
t_base = r.base;
DensoBlasting_h.MinimumMillimetersToSurface = optimise.mintargetdis*1000;
DensoBlasting_h.MaximumMillimetersToSurface = optimise.maxtargetdis*1000;
DensoBlasting_h.MinimumDegreesToSurfaceNormal = rad2deg(optimise.minDeflectionError);
DensoBlasting_h.MaximumDegreesToSurfaceNormal = rad2deg(optimise.maxDeflectionError);
DensoBlasting_h.ToolFrameReferencePoint = [0, 0, 0];
DensoBlasting_h.MinimumFunctionGradient = optimise.stol;

% Angle requiring collision checking if it changes more than this
collchk4eyeinhand_ang=4*pi/180;
valid_count=0;
foundby='';

display('Starting 1st phase...............');

%% Initial Guess - VERY IMPORTANT - starting jointConfig guess could be all zeros or current Q
% jointConfig=[0 0 0 0 0 0,0];

%pad Q with zeros if needed
while length(Q)<7; Q=[Q,0];end

jointConfig=ikine_g_plane(r,plane(1).home_point, plane(1).equ, Q);
jointConfig(jointConfig'<qlimits(:,1)*0.96)=qlimits(jointConfig'<qlimits(:,1),1)*0.96;
jointConfig(jointConfig'>qlimits(:,2)*0.96)=qlimits(jointConfig'>qlimits(:,2),2)*0.96;
if ~check_path_for_col(jointConfig)
    jointConfig=Q;
end

%% Go through each target plane passed in 
% determine pose using previous found poses 
for i = 1:length(plane)
    try 
        %give the goal to Eyeinhand pose selection
        DensoBlasting_h.TargetPoint = plane(i).home_point*1000;
        %The normal vector must be pointing away for DensoBlasting_h to
        %work properly, so assuming we wish to blast in the direction where
        %the normal is pointing towards the origin which is the base center
        %(0,0,0)
        if norm(plane(i).home_point+plane(i).equ(1:3))>norm(plane(i).home_point-plane(i).equ(1:3))
            DensoBlasting_h.TargetNormal = -plane(i).equ(1:3);
        else
            DensoBlasting_h.TargetNormal = plane(i).equ(1:3);
        end
%         plot3([plane(i).home_point(1),plane(i).home_point(1)+plane(i).equ(1)],[plane(i).home_point(2),plane(i).home_point(2)+plane(i).equ(2)],[plane(i).home_point(3),plane(i).home_point(3)+plane(i).equ(3)],'r')
%         plot3([plane(i).home_point(1),plane(i).home_point(1)-plane(i).equ(1)],[plane(i).home_point(2),plane(i).home_point(2)-plane(i).equ(2)],[plane(i).home_point(3),plane(i).home_point(3)-plane(i).equ(3)],'b')
        
        %use previous pose as the guess use DensoBlasting_h method
        jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig(1:6)))),0];
%         plot(r,jointConfig_temp)
%         DensoBlasting_h.OptimiserInfo
%         rad2deg(jointConfig_temp(1:6)-jointConfig(1:6))
        
        %do we need a collision check? how close to previous guess
        docollcheck=~isempty(find(jointConfig(1:5)-jointConfig_temp(1:5)>collchk4eyeinhand_ang,1));
        %check if valid: eg close to target goal, no collision, within limits etc
        [valid,dist]=classunkcheck_newQ(jointConfig_temp,qlimits,plane(i).home_point,t_base,Links,numlinks,plane(i).equ,false,docollcheck);
        
        %if not valid use my blasting_posesel method
        if valid
            foundby='. Found by 1) DensoBlasting_h ';
        else
            [jointConfig_temp,valid] = blasting_posesel(plane(i).home_point, plane(i).equ, jointConfig_temp,false);
            if valid 
                jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig_temp(1:6)))),0];
                foundby='. Found by 2) blasting_posesel';
            end            
        end
    catch
        if displayon
            display(['Error in pose selection of plane', num2str(i)]);
        end
        valid=false;
        jointConfig_temp=jointConfig;
    end
    
    %if not valid we don't want to use that found joint angle because it is crap
    if valid
        jointConfig=jointConfig_temp;
        
%     else
%         plot3(plane(i).home_point(1),plane(i).home_point(2),plane(i).home_point(3),'k*');if rand >0.95 drawnow;end
        
%% Used to go through and use a few successes poses as an intial guess 
% but it takes a long time and is only sometimes successful. It is better
% to do this process at the end once all sucessess have been found.

%         %go back through previous poses and try and use one of these as
%         %a starter (don't use the latest which is obviously jointConfig
%         %and has already been tried
%         tries=0;
%         if exist('pose','var') && size(pose,2)>1
%             for current_pose=randperm(size(pose,2)-1)
%                 %don't attempt to use too many poses this may be an
%                 %impossible pose after all
%                 if tries>8; break; else tries=tries+1;end
% 
%                 if pose(current_pose).validPose
%                     try 
%                         [jointConfig_temp,valid] = blasting_posesel(plane(i).home_point, plane(i).equ, pose(current_pose).Q);
%                         if valid jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig_temp(1:6)))),0];end
%                     end
%                     if valid
%                         display('Found pose using PREVIOUS VALID as starter');
%                         jointConfig=jointConfig_temp;
%                         break;
% %                         else
% %                             display('DIDNT find pose using previous valid pose as starter');
%                     end
%                 end
%             end            
%         end

%% Trying the Ikine as initial guess never seems to do much
%         if ~valid
%             %try Ikine method for starter pose
%             if ~exist('ikine_jointConfig_temp','var')
%                 ikine_jointConfig_temp=ikine_g_plane(r,plane(i).home_point, plane(i).equ, jointConfig); 
%             end
%             %use previous pose as the guess
%             jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig(1:6)))),0];
%             %check if valid
%             [valid]=classunkcheck_newQ(jointConfig_temp,qlimits,plane(i).home_point,t_base,Links,numlinks,plane(i).equ,false);
%             if ~valid            
%                 try 
%                     [jointConfig_temp,valid] = blasting_posesel(plane(i).home_point, plane(i).equ, jointConfig_temp);
%                     if valid jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig_temp(1:6)))),0];end
%                 end
%             end
%             
%             if valid
%                 display('Found a pose in hard spot using IKINE starter');
%                 jointConfig=jointConfig_temp;
%             end
%         end
    end

    % This should never happen but it must be stopped if it does
    if ~isempty(find(isnan(jointConfig),1))
        keyboard
    end
    
    %if we have to create a variable do so
    if ~exist('pose','var')
        pose(1).Q = jointConfig;
    else
        pose(end+1).Q = jointConfig;
    end 
    
    %set the pose state to valid state eg: TRUE or FALSE
    pose(end).validPose = valid;
    
    % Increase the valid pose with either TRUE (1) or FALSE (0)
    valid_count=valid_count+valid;
    %clear the found by string 
    if ~valid; foundby='';end
    %display the current status of the 1st phase of the search
    if displayon
        display([num2str(i),' of ', num2str(length(plane)),' (', num2str(i/length(plane)*100), '%) - valid: ', num2str(valid), ' (',num2str(valid_count/i*100),'% success)', foundby]);
    end
end
display('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
display(['Finished 1st search phase. Found ', num2str(valid_count), ' of ' num2str(i), ' targets (',num2str(valid_count/i*100),'%)']);

%% Go to second phase 
% In this we go through all the successful paths and see if they can be
% used to seed any of the unsucsessful ones. We go through and use future
% planes poses found to generate impossible ones 
display('Starting 2nd phase...............');
for i=1:length(plane)
    %display('only doing 60');
    if ~pose(i).validPose
        [jointConfig,valid,foundby]=tryuseallasstarters(pose,plane(i),qlimits,t_base,Links,numlinks,collchk4eyeinhand_ang);
        
        %increase by if valid is true or false
        valid_count=valid_count+valid;
        
        % If we found a valid pose then update the Q value and the
        % cumulative valid_count
        if valid 
            if displayon
                display(['Rechecking:' ,num2str(i),' of ', num2str(length(plane)),' (', num2str(i/length(plane)*100), '%) - valid: ', num2str(valid), ' (',num2str(valid_count/length(plane)*100),'% success)', foundby]);
            end
            pose(i).Q=jointConfig;
            pose(i).validPose = true;
        end
    end
end
display('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
display(['Finished 2nd search phase. Found ', num2str(valid_count), ' of ' num2str(length(plane)), ' targets (',num2str(valid_count/length(plane)*100),'%)']);

%turn warnings back on
if ~displayon
    warning on
end



%% This fucntion tries to find missing poses
function [jointConfig,valid,foundby]=tryuseallasstarters(pose,plane,qlimits,t_base,Links,numlinks,collchk4eyeinhand_ang,maxtries)
global DensoBlasting_h
if nargin<8    
    maxtries=30;
end
valid=false;
%In case we don't find a valid one
jointConfig=[0,0,0,0,0,0,0];
foundby='';

DensoBlasting_h.TargetPoint = plane.home_point*1000;
if norm(plane.home_point+plane.equ(1:3))>norm(plane.home_point-plane.equ(1:3))
    DensoBlasting_h.TargetNormal = -plane.equ(1:3);
else
    DensoBlasting_h.TargetNormal = plane.equ(1:3);
end
                        
%Try all valid poses as starter for this one missing using DensoBlasting_h
tries=0;
if size(pose,2)>1
    %it is better to use the randomised order of the poses
    for current_pose=randperm(size(pose,2)-1)
        if pose(current_pose).validPose
            try                      
                jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(pose(current_pose).Q(1:6)))),0];
                %do we need a collision check? (if greater than 3
                %deg movement)
                docollcheck=~isempty(find((pose(current_pose).Q(1:6)-jointConfig_temp(1:6))>collchk4eyeinhand_ang,1));
                %check if valid
                [valid,dist]=classunkcheck_newQ(jointConfig_temp,qlimits,plane.home_point,t_base,Links,numlinks,plane.equ,false,docollcheck);
            catch
                keyboard
            end
            % If we found then display the method used: DensoBlasting_h
            if valid
                foundby='. Found by 1) DensoBlasting_h';
                jointConfig=jointConfig_temp;
                return;
            end
        end
    end            
end

%go back through previous poses and try and some of these 60 as
%a starter (don't use the latest which is obviously jointConfig
%and has already been tried
tries=0;
if size(pose,2)>1
    for current_pose=randperm(size(pose,2)-1)       
        if pose(current_pose).validPose
            %don't attempt to use too many poses this may be an impossible pose after all        
            if tries>maxtries; break; else tries=tries+1;end
            try 
                [jointConfig_temp,valid] = blasting_posesel(plane.home_point, plane.equ, pose(current_pose).Q);
                if valid jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig_temp(1:6)))),0];end

            end
            % If we found then display the method used: blasting_posesel
            if valid
                foundby='. Found by 2) blasting_posesel';
                jointConfig=jointConfig_temp;
                return;
            end
        end
    end            
end