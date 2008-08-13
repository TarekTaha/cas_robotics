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
% _hgjdehg_
%
% *These globals are Saved to exploration_out.mat*
%

function standalone_explore()

clear all

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

%clear the globals for scan, workspace, robot(r,Q), bestviews, PointData, RangeData
clear global workspace scan bestviews r PointData RangeData guiglobal densoobj all_views robot_maxreach classunkn_optimise alldirectedpoints graf_obs;
clear global IntensityData PoseData classifiedplotHa linkFromLaserTransform optimise plane   
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
    end
end

%% TEST 2 - Non points -using algorithm
%now go through and get NBV and then use them to explore
for stepcount=stepcount+1:15;
%% determine NBV    
  NBV_beta2();
  current_bestview=1;
  max_bestviews_togothrough=optimise.valid_max*1/4;
  global bestviews;        
  while current_bestview<max_bestviews_togothrough && size(bestviews,2)>=1
      current_bestview=current_bestview+1;
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
                      scan.done_bestviews_orfailed=[scan.done_bestviews_orfailed;bestviews(1).Q];explore(handles,useNBV,1);validpathfound=true;
                      break;
                  end
              end             
          catch; display(lasterr);keyboard; 
          end;            
      end                
      %termination conditions
      changeinweight=diff(state_data.knownweight);
      if length(changeinweight)>3
          if sum(changeinweight(end-3:end))<100
              %set to stop ASAP
              want_to_continue=false;
              display('Termination condition reached');
              break;
          end
      end

      if size(bestviews,2)>1  
          %Resize bestviews so as to get rid of the first element
          clear temp_bestviews;
          for cur_view=2:size(bestviews,2); temp_bestviews(cur_view-1)=bestviews(cur_view);end
          bestviews=temp_bestviews;
          order_bestviews(true);
      else
          bestviews=[];
      end
  end
end

%% Save globals
keyboard
robmap_h
workspace
r


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
UNclassifiedvoxels=update_ocstatus(ClassifiedData); 