%Note: This assumes we wish to blast in the direction where the normal
%is pointing towards the origin which is the base center (0,0,0)
        
%NOTE: it is assumed that currQ (and or Q) is safe 
function [pose]=PoseSel4planesearch(plane,displayon,currQ,useMyMethodOnly)
global r optimise jointConfigCollisionChecked Q allE

if nargin<4
  %if this is true we use both mine and steves
  useMyMethodOnly=false;
  if nargin<3
      currQ=Q;
      if nargin<2
          displayon=true;
      end
  end
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
%             DensoBlasting_h = robmap_h.GetDensoBlastingCostFunction;
            DensoBlasting_h = actxserver('EyeInHand.DensoBlastingCost');
        end
    catch
        display('EyeInHand Problem: Unable to create DensoBlastingCost from surface map')
    end
catch
    display('EyeInHand Problem: Unable to create surface map')
end

%% Variables 
numlinks = r.n;

% we must setup the robot again to be a 7 link one otherwise the pose selection wont work
if numlinks<7
    tempQ=Q;
    setuprobot(7);
    make6jointrobot=true;
    %pad currQ with zeros if needed
    while length(currQ)<7; currQ=[currQ,0];end
    %uppdate the number of links
    numlinks = r.n;
else
    make6jointrobot=false;
end
%Should pass in a currQ with the same size as the robot and =7
if size(currQ,2)<7
	currQ=[currQ,0];
end




    
Links = r.link;
qlimits=r.qlim; 
t_base = r.base;
try DensoBlasting_h.MinimumMillimetersToSurface = optimise.mintargetdis*1000;
catch
  display('PoseSel4planesearch:: trying to restart the server: EyeInHand.DensoBlastingCost')
  DensoBlasting_h = actxserver('EyeInHand.DensoBlastingCost');
  DensoBlasting_h.MinimumMillimetersToSurface = optimise.mintargetdis*1000;
end
DensoBlasting_h.MaximumMillimetersToSurface = optimise.maxtargetdis*1000;
DensoBlasting_h.MinimumDegreesToSurfaceNormal = rad2deg(optimise.minDeflectionError);
DensoBlasting_h.MaximumDegreesToSurfaceNormal = rad2deg(optimise.maxDeflectionError);
DensoBlasting_h.ToolFrameReferencePoint = [0, 0, 0];
DensoBlasting_h.MinimumFunctionGradient = optimise.stol;

%this will store where collision checks have been done
jointConfigCollisionChecked=currQ;
        
for piece=1:numlinks
	linkvals(piece).val=[Links{piece}.alpha Links{piece}.A Links{piece}.D Links{piece}.offset];
end
    
% Angle requiring collision checking if it changes more than this
collchk4eyeinhand_ang=2*pi/180;
valid_count=0;
foundby='';

display('Starting 1st phase...............');

%% Initial Guess - VERY IMPORTANT - starting jointConfig guess could be all zeros or current Q
% jointConfig=[0 0 0 0 0 0,0];



jointConfig=ikine_g_plane(r,plane(1).home_point, plane(1).equ, currQ);
jointConfig(jointConfig'<qlimits(:,1)*0.96)=qlimits(jointConfig'<qlimits(:,1)*0.96,1)*0.96;
jointConfig(jointConfig'>qlimits(:,2)*0.96)=qlimits(jointConfig'>qlimits(:,2)*0.96,2)*0.96;
if ~check_path_for_col(jointConfig)
    jointConfig=currQ;
else %its ok so add to list of checks done
    jointConfigCollisionChecked=[jointConfigCollisionChecked;jointConfig];
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
        if ~useMyMethodOnly
          jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig(1:6)))),0];
        else
          jointConfig_temp=jointConfig;
        end
%         plot(r,jointConfig_temp)
%         DensoBlasting_h.OptimiserInfo
%         rad2deg(jointConfig_temp(1:6)-jointConfig(1:6))
        
        %do we need a collision check? how close to previous guess
        %before all classunknown_newQ checks    
        %look for a nearby joint angle that has been already checked. If
        %there is a nearby angle then DONT do a collision check
        docollcheck=isempty(find((jointConfigCollisionChecked(:,1)-jointConfig_temp(1)<collchk4eyeinhand_ang & ...
                                  jointConfigCollisionChecked(:,2)-jointConfig_temp(2)<collchk4eyeinhand_ang & ...
                                  jointConfigCollisionChecked(:,3)-jointConfig_temp(3)<collchk4eyeinhand_ang & ...
                                  jointConfigCollisionChecked(:,4)-jointConfig_temp(4)<collchk4eyeinhand_ang & ...
                                  jointConfigCollisionChecked(:,5)-jointConfig_temp(5)<collchk4eyeinhand_ang),1));                                                            
        %check if valid: eg close to target goal, no collision, within limits etc
        [valid,dist]=classunkcheck_newQ(jointConfig_temp,qlimits,plane(i).home_point,t_base,Links,numlinks,plane(i).equ,false,linkvals,docollcheck);

        %put this after all classunknown_newQ checks and new_blasting_posesel 
        if docollcheck && valid
            jointConfigCollisionChecked=[jointConfigCollisionChecked;jointConfig_temp];
        end
        
        
%% if not valid use my new_blasting_posesel method
        if valid && ~useMyMethodOnly
            foundby='. Found by 1) DensoBlasting_h ';                        
        else
          if useMyMethodOnly
            [jointConfig_temp,valid] = new_blasting_posesel(plane(i).home_point, plane(i).equ, jointConfig_temp,true);
          else
            [jointConfig_temp,valid] = new_blasting_posesel(plane(i).home_point, plane(i).equ, jointConfig_temp,false);
          end
           
            %Store that a collision check has been performed
            if valid
                jointConfigCollisionChecked=[jointConfigCollisionChecked;jointConfig_temp];
            end
        
            if valid 
                jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig_temp(1:6)))),0];
                foundby='. Found by 2) new_blasting_posesel';
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
%                         [jointConfig_temp,valid] = new_blasting_posesel(plane(i).home_point, plane(i).equ, pose(current_pose).Q);
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
%                     [jointConfig_temp,valid] = new_blasting_posesel(plane(i).home_point, plane(i).equ, jointConfig_temp);
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
    %if this is not set it will be empty
    pose(end).allE=allE;
    %make sure it is empty
    allE.E=[];
    allE.Q=[];
    
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
all_valid=zeros([valid_count,numlinks]);
valid_count=0;
for i=1:length(plane)
    if pose(i).validPose
        valid_count=valid_count+1;
        all_valid(valid_count,:)= pose(i).Q;
    end
end
mew=sum(all_valid,1)/valid_count;
disttomean=sqrt((all_valid(:,1)-mew(1)).^2+(all_valid(:,2)-mew(2)).^2+(all_valid(:,3)-mew(3)).^2+(all_valid(:,4)-mew(4)).^2+(all_valid(:,5)-mew(5)).^2);
[nothing,index]=sortrows(disttomean);
% all_valid(index,:);

for i=1:length(plane)
    %display('only doing 60');
    if ~pose(i).validPose
        [jointConfig,valid,foundby]=tryuseallasstarters(pose,plane(i),qlimits,t_base,Links,numlinks,all_valid(index,:),collchk4eyeinhand_ang,linkvals);
        
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
            %update the all_valid variable
            all_valid=[all_valid;jointConfig];
            %update the mean
            mew=(mew*(valid_count-1)+jointConfig)/valid_count;
            %get distance to new mean
            disttomean=sqrt((all_valid(:,1)-mew(1)).^2+(all_valid(:,2)-mew(2)).^2+(all_valid(:,3)-mew(3)).^2+(all_valid(:,4)-mew(4)).^2+(all_valid(:,5)-mew(5)).^2);
            %sort in order of distance from mean
            [nothing,index]=sortrows(disttomean);
        end
    end
end
display('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
display(['Finished 2nd search phase. Found ', num2str(valid_count), ' of ' num2str(length(plane)), ' targets (',num2str(valid_count/length(plane)*100),'%)']);

%turn warnings back on
if ~displayon
    warning on
end

%if we changed the robot for blast planning then change it back
if make6jointrobot
    setuprobot(6);
    %put Q back to what it was
    Q=tempQ;
    for i=1:length(plane)
       if pose(i).validPose
           pose(i).Q=pose(i).Q(:,1:6);
       end
    end    
end



%% This fucntion tries to find missing poses
function [jointConfig,valid,foundby]=tryuseallasstarters(pose,plane,qlimits,t_base,Links,numlinks,Q_guess_order,collchk4eyeinhand_ang,linkvals,maxtries)
global DensoBlasting_h jointConfigCollisionChecked
if nargin<10    
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
if size(Q_guess_order,1)>1
    %it is better to use the randomised order of the poses
%     for current_pose=randperm(size(pose,2)-1)
% if pose(current_pose).validPose
%using the closest guesses to the mean
    for current_pose=1:size(Q_guess_order,1)
        try                      
            jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(Q_guess_order(current_pose,1:6)))),0];
%                 jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(pose(current_pose).Q(1:6)))),0];


            %do we need a collision check? how close to previous guess
            %before all classunknown_newQ checks    
            %look for a nearby joint angle that has been already checked. If
            %there is a nearby angle then DONT do a collision check
            docollcheck=isempty(find((jointConfigCollisionChecked(:,1)-jointConfig_temp(1)<collchk4eyeinhand_ang & ...
                                      jointConfigCollisionChecked(:,2)-jointConfig_temp(2)<collchk4eyeinhand_ang & ...
                                      jointConfigCollisionChecked(:,3)-jointConfig_temp(3)<collchk4eyeinhand_ang & ...
                                      jointConfigCollisionChecked(:,4)-jointConfig_temp(4)<collchk4eyeinhand_ang & ...
                                      jointConfigCollisionChecked(:,5)-jointConfig_temp(5)<collchk4eyeinhand_ang),1));                                                            
               
             %check if valid
            [valid,dist]=classunkcheck_newQ(jointConfig_temp,qlimits,plane.home_point,t_base,Links,numlinks,plane.equ,false,linkvals,docollcheck);
            %put this after all classunknown_newQ checks and new_blasting_posesel 
            if docollcheck && valid
                jointConfigCollisionChecked=[jointConfigCollisionChecked;jointConfig_temp];
            end
        
        catch
            keyboard
        end
            % If we found then display the method used: DensoBlasting_h
        if valid
            foundby='. Found by 1) DensoBlasting_h';
            jointConfig=jointConfig_temp;
            return;
        end
%         end
    end            
end

%go back through previous poses and try and some of these 60 as
%a starter (don't use the latest which is obviously jointConfig
%and has already been tried
tries=0;
if size(Q_guess_order,1)>1
    for current_pose=1:size(Q_guess_order,1)
%     for current_pose=randperm(size(pose,2)-1)       
%         if pose(current_pose).validPose
            %don't attempt to use too many poses this may be an impossible pose after all        
        if tries>maxtries; break; else tries=tries+1;end
        try 
            [jointConfig_temp,valid] = new_blasting_posesel(plane.home_point, plane.equ, Q_guess_order(current_pose,:));
                                
            if valid 
                %Store that a collision check has been performed
                jointConfigCollisionChecked=[jointConfigCollisionChecked;jointConfig_temp];
                jointConfig_temp =[deg2rad(DensoBlasting_h.MinimumNear(rad2deg(jointConfig_temp(1:6)))),0];
            end
        end
            % If we found then display the method used: new_blasting_posesel
        if valid
            foundby='. Found by 2) new_blasting_posesel';
            jointConfig=jointConfig_temp;
            return;
        end
    end
%     end            
end