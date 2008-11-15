function [pose]=PoseSel4planesearch(plane)
global r Q optimise

%setup the pose sel object
try global robmap_h; 
    try robmap_h.release;pause(1);end
robmap_h=actxserver('EyeInHand.SurfaceMap');
robmap_h.registerevent(@myhandler);
    %get the Denso Blasting Cost Function
    try global DensoBlasting_h
        DensoBlasting_h = robmap_h.GetDensoBlastingCostFunction;
    catch
        display('EyeInHand Problem: Unable to create DensoBlastingCost from surface map')
    end
catch
    display('EyeInHand Problem: Unable to create surface map')
end

%added since classunkcheck_new was added as an available method
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
    
tic

%   profile clear; profile on;
display('Currently determining poses for target set');

%starting jointConfig guess could be all zeros or current Q
% jointConfig=[0 0 0 0 0 0,0];
jointConfig=ikine_g_plane(r,plane(1).home_point, plane(1).equ, [0 0 0 0 0 0,0]);
jointConfig(jointConfig'<qlimits(:,1))=qlimits(jointConfig'<qlimits(:,1),1);
jointConfig(jointConfig'>qlimits(:,2))=qlimits(jointConfig'>qlimits(:,2),2);


    
for i = 1:length(plane)
    try 
        %guve the goal to Eye in hand pose selection
        DensoBlasting_h.TargetPoint = plane(i).home_point*1000;
        DensoBlasting_h.TargetNormal = plane(i).equ(1:3);
        %use previous pose as the guess
        jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig(1:6)))),0];
        %check if valid
        [valid,dist]=classunkcheck_newQ(jointConfig_temp,qlimits,plane(i).home_point,t_base,Links,numlinks,plane(i).equ,false);
        
%         DensoBlasting_h.OptimiserInfo        
        %if not valid
        if ~valid
            %if it came out close then use as guess for matlab version
%             if dist<0.05
                [jointConfig_temp,valid] = blasting_posesel(plane(i).home_point, plane(i).equ, jointConfig_temp,true);
                if valid jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig_temp(1:6)))),0];end
%             else
%                 %use the ikine guess
%                 ikine_jointConfig_temp=ikine_g_plane(r,plane(i).home_point, plane(i).equ, jointConfig); 
%                 jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(ikine_jointConfig_temp(1:6)))),0];
%                 [valid,dist]=classunkcheck_newQ(jointConfig_temp,qlimits,plane(i).home_point,t_base,Links,numlinks,plane(i).equ,true);
%                 %if still not valid use the jointConfig_temp for matlab pose selection
%                 if ~valid
% %                     dist
%                     [jointConfig_temp,valid] = blasting_posesel(plane(i).home_point, plane(i).equ, jointConfig_temp);
%                     if valid jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig_temp(1:6)))),0];end
%                 end
%             end
        else
            display('Found a pose immediately using EYEINHAND');
        end
        
        
    catch        
        display(['Error in pose selection of plane', num2str(i)]);
        valid=false;
        jointConfig_temp=jointConfig;
    end
    
    
    
    %if not valid we don't want to use that found joint angle because it is crap
    if valid
        jointConfig=jointConfig_temp;
    else
         plot3(plane(i).home_point(1),plane(i).home_point(2),plane(i).home_point(3),'k*');
        if rand >0.95 drawnow;end
        
        %go back through previous poses and try and use one of these as
        %a starter (don't use the latest which is obviously jointConfig
        %and has already been tried
        tries=0;
        if exist('pose','var') && size(pose,2)>1
            for current_pose=randperm(size(pose,2)-1)
                %don't attempt to use too many poses this may be an
                %impossible pose after all
                if tries>8; break; else tries=tries+1;end

                if pose(current_pose).validPose
                    try 
                        [jointConfig_temp,valid] = blasting_posesel(plane(i).home_point, plane(i).equ, pose(current_pose).Q);
                        if valid jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig_temp(1:6)))),0];end
                    end
                    if valid
                        display('Found pose using PREVIOUS VALID as starter');
                        jointConfig=jointConfig_temp;
                        break;
%                         else
%                             display('DIDNT find pose using previous valid pose as starter');
                    end
                end
            end            
        end
        if ~valid
            %try Ikine method for starter pose
            if ~exist('ikine_jointConfig_temp','var')
                ikine_jointConfig_temp=ikine_g_plane(r,plane(i).home_point, plane(i).equ, jointConfig); 
            end
            %use previous pose as the guess
            jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig(1:6)))),0];
            %check if valid
            [valid]=classunkcheck_newQ(jointConfig_temp,qlimits,plane(i).home_point,t_base,Links,numlinks,plane(i).equ,false);
            if ~valid            
                try 
                    [jointConfig_temp,valid] = blasting_posesel(plane(i).home_point, plane(i).equ, jointConfig_temp);
                    if valid jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig_temp(1:6)))),0];end
                end
            end
            
            if valid
                display('Found a pose in hard spot using IKINE starter');
                jointConfig=jointConfig_temp;
            end
        end
    end

    
    if ~isempty(find(isnan(jointConfig),1))
        keyboard
    end
    
    %if we have to create a variable do so
    if ~exist('pose','var')
        pose(1).Q = jointConfig;
    else
        pose(end+1).Q = jointConfig;
    end 
    
    %set the pose state to valid state TRUE or FALSE
    pose(end).validPose = valid;
    
    
    display([num2str(i),' of ', num2str(length(plane)),' (', num2str(i/length(plane)*100), '%) - valid: ', num2str(valid)]);
end

%Go through and use future planes poses found to generate impossible ones
for i=1:length(plane)
    display('only doing 20');
    if ~pose(i).validPose
        [jointConfig,valid]=tryuseallasstarters(pose,plane(i));
        if valid
            pose(i).Q=jointConfig;
            pose(i).validPose = true;
        end
    end
end
        


%% This fucntion tries to find missing poses
function [jointConfig,valid]=tryuseallasstarters(pose,plane,maxtries)
global DensoBlasting_h
if nargin==2
    
    maxtries=50;
end
valid=false;jointConfig=[];

        %Try all valid poses as starter for this one missing using
        %DensoBlasting_h
        tries=0;
        if size(pose,2)>1
            for current_pose=randperm(size(pose,2)-1)
                %don't attempt to use too many poses this may be an
                %impossible pose after all
                if tries>maxtries; break; else tries=tries+1;end

                if pose(current_pose).validPose
                    try 
                        DensoBlasting_h.TargetPoint = plane.home_point*1000;
                        DensoBlasting_h.TargetNormal = plane.equ(1:3);
        
                        jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(pose(current_pose).Q(1:6)))),0];
                        [valid,dist]=classunkcheck_newQ(jointConfig_temp,qlimits,plane(i).home_point,t_base,Links,numlinks,plane(i).equ,false);
                    end
                    if valid
                        display('Found pose using DensoBlasting_h & PREVIOUS VALID as starter');
                        jointConfig=jointConfig_temp;
                        return;
                    end
                end
            end            
        end
        
        %go back through previous poses and try and use one of these as
        %a starter (don't use the latest which is obviously jointConfig
        %and has already been tried
        tries=0;
        if size(pose,2)>1
            for current_pose=randperm(size(pose,2)-1)
                %don't attempt to use too many poses this may be an
                %impossible pose after all
                if tries>maxtries; break; else tries=tries+1;end

                if pose(current_pose).validPose
                    try 
                        [jointConfig_temp,valid] = blasting_posesel(plane.home_point, plane.equ, pose(current_pose).Q);
                        if valid jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig_temp(1:6)))),0];end

                    end
                    if valid
                        display('Found pose using blasting_posesel & PREVIOUS VALID as starter');
                        jointConfig=jointConfig_temp;
                        return;
                    end
                end
            end            
        end

%If we havene't set jointConfig yet and it is invalid then set it
if ~valid
    jointConfig=[0,0,0,0,0,0,0];
end