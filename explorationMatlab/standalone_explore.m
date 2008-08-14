%% standalone_explore
%
% *Description:* This is a hacked version of Gavin's exploration

%% Function Call
%
% *Inputs:* NULL
%
% *Returns:* NULL passed
%
% *Globals still in memory*
%
% _robmap_h_ (stephens surface map ofject)
%
% _platform_h_ (stephens platform ofject)
%
% *These globals are Saved to exploration_out.mat*
%
% _verts_ (extracted from robmap_h which is still available
% 

function standalone_explore()

clear all;close all;

%MANUAL CHECKS: To do before start work
display('Make sure you have done the following');
display('1) Checked the laser is on by running the checklaser.exe file (power cycle if necessary)');
display('2) The ROBOT is powered on');
display('3) The MOTOR power is on');
display('4) Both EMERGENCY STOPs are not on');
display('5) Robot is in automatic mode on both pendant and E-stop control');
display('6) The robocontroller program is running on pendant and recieveing no communication');
display('7) There is no EyeinHand exe running intially in task manager');

display('Pausing so you can do these things');pause


%% SETUP: the surface map object and then this is used for scanning
try global robmap_h;
    try robmap_h.release;pause(1);end
robmap_h=actxserver('EyeInHand.SurfaceMap');
robmap_h.registerevent(@myhandler);
catch
    display('EyeInHand Problem: Unable to create surface map')
end

% try and setup platform object
try global platform_h;
    try platform_h.release;pause(1);end
    platform_h = actxserver('EyeInHand.PlatformCommand'); 
catch
    display('EyeInHand Problem: Unable to create platform object')
end

% Used to continually monitor the joint state
DensoState_h = actxserver('EyeInHand.DensoState');
DensoState_h.registerevent(@updateQlistener);

%% Clear all globals I use
%clear the overall seetup globals 
clear global workspace scan bestviews r densoobj all_views robot_maxreach optimise;
%clear globals for data from laser
clear global IntensityData PoseData PointData RangeData 
% Clear other globals
clear global classunkn_optimise classifiedplotHa plane alldirectedpoints graf_obs

%% Setup functions
%Sets up the robot
setuprobot()

% Sets up the scanner (scan)
setupscanner();

%this sets up the joint optimisation and NBV optimisation 
setupoptimisation()

% Setupworkspace
setupworkspace(false);

%so we don't need the GUI handles
NOhandleOPTIONS=struct('useRealRobot',true,'show_robot',false,'animate_move',false,'remv_unkn_in_mv',true);

display('Setup Complete: Lets Explore');

%% EXPLORATION
global robot_maxreach optimise workspace scan Q

% do we want to continue or stop
want_to_continue=true;

%hack for standalone to fix termination conditions
changeinweight=size(workspace.knowncoords,1);
  
%% intial safe pose default scans
%since we wish to choose our own default start points
for stepcount=2:size(robot_maxreach.default_Q,1)+1
    while want_to_continue && scan.tries<size(robot_maxreach.default_Q,1)
      %% Move to newQ
      newQ=robot_maxreach.default_Q(scan.tries+1,:);
      movetonewQ([],rad2deg(newQ),[],NOhandleOPTIONS);
      try standalone_scanNupdate(newQ)        
      catch display(lasterr);keyboard;
      end
      changeinweight=[changeinweight;size(workspace.knowncoords,1)];
    end
end

%% Actual NBV determination and Exploration
% go through and get NBV and then use them to explore
for stepcount=stepcount+1:10;
%% determine NBV    
  NBV_beta2();
  %   %this needs to be here since it is cleared in NBV_beta2()
  global bestviews;        
      while want_to_continue; 
          try %if we have already planned a path, use this one otherwise try and get another, otherwise go to next possible one
              if movetonewQ([],rad2deg(bestviews(1).Q),bestviews(1).all_steps,NOhandleOPTIONS);
                  scan.done_bestviews_orfailed=[scan.done_bestviews_orfailed;bestviews(1).Q];
                  standalone_scanNupdate(bestviews(1).Q);
                  validpathfound=true;
                  break;
              else %can't get to the desired best view
                  display('User has control');
                  keyboard

                  %tac on the actual position here just in case it isn't exactly where it was supposed to finish
                  robot_maxreach.path(end).all_steps(end+1,:)=Q;
                  %move back along the path taken to get here
                  if ~movetonewQ([],rad2deg(robot_maxreach.path(end).all_steps(1,:)),robot_maxreach.path(end).all_steps(end:-1:1,:),NOhandleOPTIONS);
                      display('some major problem if we cant follow the same path back');
                      keyboard                                
                  end
                  %try once again to move to the actual desired newQ for exploration
                  if movetonewQ([],rad2deg(bestviews(1).Q),bestviews(1).all_steps,radNOhandleOPTIONS);
                      scan.done_bestviews_orfailed=[scan.done_bestviews_orfailed;bestviews(1).Q];
                      standalone_scanNupdate(bestviews(1).Q);
                      validpathfound=true;
                      break;
                  end
              end             
          catch; display(lasterr);keyboard; 
          end;            
      end
      
      %Check termination conditions
      changeinweight=[changeinweight;size(workspace.knowncoords,1)];
      if length(changeinweight)>3
          if sum(changeinweight(end-3:end))<300
              %set to stop ASAP
              want_to_continue=false;
              display('Termination condition reached');
              break;
          end
      end     
end

%% Plot nearby map
try aabb = [-2, -2, -2; 2, 2, 2];hMesh = robmap_h.Mesh(aabb);f = hMesh.FaceData;v = hMesh.VertexData;hold on;guiglobal.mesh_h=trisurf(f, v(:,1), v(:,2), v(:,3), 'FaceColor', 'None');end
try global r; hold on;plotdenso(r,Q);light;end

%% Save important variables
display('Saving vertices in -2 to 2 cube');
save(['standalone_explore_pointcloud',date,'-',num2str(rand),'.mat'],'v');
display('Saving workspace including freespace and classification results');
save(['standalone_explore_workspace',date,'-',num2str(rand),'.mat'],'workspace');

keyboard

%% Delete all my globals from memory
%clear the overall seetup globals 
clear global workspace scan bestviews r densoobj all_views robot_maxreach optimise;
%clear globals for data from laser
clear global IntensityData PoseData PointData RangeData 
% Clear other globals
clear global classunkn_optimise classifiedplotHa plane alldirectedpoints graf_obs guiglobal

%% Display Useful globals
display('Useful globals in memory are: platform_h -> EyeInHand.PlatformCommand, robmap_h -> EyeInHand.SurfaceMap).');
display('Run memmgr to find out all variables in memory');
display('------END Exploration Hack------------');






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% SEPARATE FUNCTION FOR SCAN, UPDATE MAP, CLASSIFY
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
%% Do the Scan, classify, update voxels
function standalone_scanNupdate(newQ)
global r scan         

% Change newQ so if scanning on the downside (+ angles) -> start from max place and goes to the center, 
% Alternately,it goes to +30', and then scan to the max negative possible
qlimits=r.qlim;
minimum_alpha=qlimits(5,1)*0.9;
%determine the maximum angle
if newQ(3)>scan.alpha_limited_condition
    maximum_alpha=scan.alpha_limited;
else 
    maximum_alpha=scan.alpha;
end    
%determine where to start and where to tilt through too
if scan.chosenview(3)>0
    newQ(5)=min(maximum_alpha,newQ(5)+scan.alpha/2);
    tilt_scan_range=minimum_alpha-newQ(5);
else
    newQ(5)=max(minimum_alpha,newQ(5)-scan.alpha/2);
    tilt_scan_range=maximum_alpha-newQ(5);
end

%% Move to newQ
NOhandleOPTIONS=struct('useRealRobot',true,'show_robot',false,'animate_move',false,'remv_unkn_in_mv',true);
movetonewQ([],rad2deg(newQ),[],NOhandleOPTIONS);


%% Scan and update data    

%take a scan through determined tilt_scan_range
use_real_robot_SCAN(tilt_scan_range*180/pi);
%update the latest position of robot
use_real_robot_GETJs();    
%this sort out the points that we have got from a scan
organise_data();

%% Classify    

%if we want to do the classify and update voxels then we will do this here
global PointData IntensityData RangeData;
[ClassifiedData] = Block_Classifier(PointData, IntensityData,RangeData); 
display('Doing classifications voxel update');
UNclassifiedvoxels=update_ocstatus(ClassifiedData); 
display('Finshed update');

%% Update scan tries
scan.tries=scan.tries+1;