%% movetonewQ
%
% *Description:* either passed or got from the GUI, updates the GUI
% move to new Q function for GUI plotting and sending commands
% requires exGUI to run.
% Special Note: newQ IS IN DEGREES!!

%% Function Call
%
% *Inputs:* 
%
% _handles_ (array double) GUI variables (pass '0' if you dont have one)
%
% _newQ_(1*6 double) newQ in DEGREES
%
% _all_steps_ (many*6) path of all 6 joints in RADs
%
% _NOhandleOPTIONS_ (Struc) MUST FILL IN ALL FIELDS:
% useRealRobot, show_robot, animate_move, remv_unkn_in_mv
%
% _hFigure_ (Struc) This is the handle which can be passed on so as to
% update the RTA project figure
%
% *Returns:* 
%
% _pathfound_ (binary) if a path was found or not

function pathfound=movetonewQ(handles,newQ,all_steps,NOhandleOPTIONS,hFigure)

%% Variables
global r Q workspace robot_maxreach;

%if there is a figure then try and get guiParams from it
fig_H=get(0,'CurrentFigure');
if ~isempty(fig_H)
    guiParams=getappdata(fig_H,'guiParams');
else
    guiParams=[];
end

% check we have got passed a newQ otherwise use from the GUI
if nargin<4
    if isstruct(handles)==0
        error('You must pass the handle to exGUI or set boolean parameters (useRealRobot, show_robot, animate_move, remv_unkn_in_mv) in NOhandleOPTIONS');
    end
    if nargin<3
        all_steps=[];
        if nargin<2
            %get the desired destination from the GUI
            newQ(1)=str2double(get(handles.move_to_J1_edit,'String'));
            newQ(2)=str2double(get(handles.move_to_J2_edit,'String'));
            newQ(3)=str2double(get(handles.move_to_J3_edit,'String'));
            newQ(4)=str2double(get(handles.move_to_J4_edit,'String'));
            newQ(5)=str2double(get(handles.move_to_J5_edit,'String'));
            newQ(6)=str2double(get(handles.move_to_J6_edit,'String'));
            if find(isnan(newQ))>0
                error('Some values you are requesting to move to are not valid - all must be numbers')
            end
        end
    end
end

if size(all_steps,1)>0 && size(newQ,2)~=size(all_steps,2)
  error('newQ and all_steps variables must be the same size');
end
  
%changing the size since we only 
if size(newQ,2)>r.n
  newQ=newQ(1:6);
  if size(all_steps,1)>0
    all_steps=all_steps(:,1:6);
  end
end

  
if isstruct(guiParams); plotpath=guiParams.plotpath; else plotpath=false;end

%if we don't have any handles
if isstruct(handles)==0
    display('Handles not passed in so not updating the GUI');
    scanwhilemove=false;
    tryalternate=true;
    if NOhandleOPTIONS.useRealRobot; useRealRobot=true;else useRealRobot=false;end
    if NOhandleOPTIONS.show_robot; show_robot=true;else show_robot=false;end
    if NOhandleOPTIONS.animate_move; animate_move=true;else animate_move=false;end
    if NOhandleOPTIONS.remv_unkn_in_mv; remv_unkn_in_mv=true;else remv_unkn_in_mv=false;end    
    %whether to plot paths in water path planner or not    
else
    scanwhilemove=get(handles.scanwhilemove_checkbox,'value');
%% Check if we only want to go to the exact destination
    if get(handles.exact_joints_only_checkbox,'Value')
        tryalternate=false;
    else
        tryalternate=true;
    end
    if 	get(handles.useRealRobot_checkbox,'Value'); useRealRobot=true; else useRealRobot=false; end
    if 	get(handles.show_robot_checkbox,'Value'); show_robot=true; else show_robot=false; end
    if 	get(handles.animate_move_checkbox,'Value'); animate_move=true; else animate_move=false; end
    if 	get(handles.remv_unkn_in_mv_checkbox,'Value'); remv_unkn_in_mv=true; else remv_unkn_in_mv=false; end    
end
    
%change to rads
newQ=newQ*pi/180;




%% Get the latest Q from the robot if we are using it
if 	useRealRobot
    use_real_robot_GETJs();
end


%% Check if we are already at destination or very close (rounding error)
%check if we are already at the destination and return if we are
%eps is a very small value and if there is less than this angular distance
%we disregard this and say it is at the same place
if isempty(find(abs(Q-newQ)>robot_maxreach.minjointres, 1));
    %since we are already at the correct path
    try set(handles.dialog_text,'String','Already at destination');drawnow;
    catch; display('Already at destination'); end
    
    if show_robot
        plotdenso(r, newQ, guiParams.checkFF, guiParams.plot_ellipse);
    end
    %since we are already there
    pathfound=1;
    return
end

%% Do path planning
% Provided we haven't passed a path in which is full and has a start at Q
% and finish at newQ, also that they are all numbers
if ~isempty(all_steps) && ...
        isempty(find(all_steps(1,1:3)-Q(1:3)>robot_maxreach.minjointres,1)) && ...
        isempty(find(all_steps(end,1:3)-newQ(1:3)>robot_maxreach.minjointres,1)) &&...
        isempty(find(isnan(all_steps),1))
    %check the path passed in for collisions
    pathfound=check_path_for_col(all_steps);
    if ~pathfound
        display('The passed in path is not valid - calculating another');
        %try and move directly to the goal with several combinations of
        %joints but with no middle points
        try [pathfound,all_steps]=pathplanner_new(newQ,false,true,false,0,false);end
        if pathfound==0
          pathval=pathplanner_water(newQ,plotpath);pathfound=pathval.result;all_steps=pathval.all_steps;
        end
    end     
else % no valid path has been passed
    try 
        tic; 
        %default is that no path is found
        pathfound=0;
        %try and move directly to the goal with several combinations of
        %joints but with no middle points
        try [pathfound,all_steps]=pathplanner_new(newQ,false,true,false,0,false);end
        if pathfound==0
          pathval=pathplanner_water(newQ,plotpath);pathfound=pathval.result;all_steps=pathval.all_steps;
        end

        display(['Additional Path Planning time is: ',num2str(toc)]);
    catch
        lasterr;display('error in path planner');
        keyboard; 
    end
end

if pathfound==0 || pathfound==-1
  if pathfound==0 %no path found, update GUI accordingly      
    if show_robot
        plotdenso(r, Q, guiParams.checkFF, guiParams.plot_ellipse);
    end
    try set(handles.dialog_text,'String','End valid but no path found');
    catch; display('End valid but no path found');end
    
    %if it is an error returned means that the end is not valid
  elseif pathfound==-1     
    try set(handles.dialog_text,'String','Impossible Path - Collision avoided! Ignoring command!');
    catch; display('Impossible Path - Collision avoided! Ignoring command!');end
  end
    
  % VERY DANGEROUS, AS LONG AS YOU ARE SURE YOU CAN MOVE THERE DIRECTLY    
  userchoice=questdlg('THIS IS VERY DANGEROUS, I CAN MOVE LINEARLY TO THE GOAL, YOU MAY HIT ENVIRONMENT, DO YOU WANT TO CONTINUE','DANGER','Yes','No','ThrowError','ThrowError');
  if strcmp(userchoice,'Yes');
    incstoadd=[1:10]'*((newQ-Q)/10);
    all_steps=[Q;...
              Q(1)+incstoadd(:,1),Q(2)+incstoadd(:,2),Q(3)+incstoadd(:,3),Q(4)+incstoadd(:,4),Q(5)+incstoadd(:,5),Q(6)+incstoadd(:,6);
              newQ];
    %overriding pathfound to make it true
    pathfound=true
  elseif strcmp(userchoice,'Error');
    error('It is not safe/possible to move here');
  end           
end

%% If there is a path then do the actual move(and plot) else do nothing
if pathfound && size(all_steps,1)>0
    %if animate has been selected it will go through each step of the path
    if animate_move
        try set(handles.dialog_text,'String','Path found - animating....');
        catch; display('Path found - animating....');end
        demopath_new(all_steps);
    end
    %plot the end effector in place
    if show_robot
        plotdenso(r, all_steps(end,:), guiParams.checkFF, guiParams.plot_ellipse);
    end
    try set(handles.dialog_text,'String','Path found - animation updated');
    catch; display('Path found - animation updated'); end
    %If using real robot try and move it
    if useRealRobot
        try if ~scanwhilemove
                % If we need to pass in the fig handle for RTA proj code
                if nargin==5 use_real_robot_MOVE(all_steps,hFigure); 
                else use_real_robot_MOVE(all_steps); 
                end
                  
                  
                try set(handles.dialog_text,'String','Actual robot movement complete');
                catch; display('Actual robot movement complete');end                    
            else
                use_real_robot_SCANandMOVE(all_steps);
                try organise_data(); end;
                try set(handles.dialog_text,'String','Scan N Move completed');
                catch; display('Scan N Move completed'); end
            end
            %save the successfull path incase we need to retrace our steps
            if (size(all_steps,1)>3 && ~isempty(find(abs(all_steps(1,1:4)-all_steps(end,1:4))>pi/180,1))) ||...
                    isempty(robot_maxreach.path)
                robot_maxreach.path(end+1).all_steps=all_steps;
            else %add the slight movement (scan+move&adjust) onto the end of last path
                robot_maxreach.path(end).all_steps=[robot_maxreach.path(end).all_steps;all_steps];
            end
            
        catch
          lasterr;
          keyboard;
          try set(handles.dialog_text,'String','Error: Did not complete movement - Emergency Stop Probably Hit');
          catch display('Error: Did not complete movement - Emergency Stop Probably Hit'); end
            lasterr;
            error('Did not complete movement');
        end
    end

    %remove obstacle points that are recorded but were in the path    
    insidepoints=[];
    for step=1:size(all_steps,1)           
        insidepoints = [insidepoints;find_points_in_FF(workspace.unknowncoords(workspace.lev1unknown,:),all_steps(step,:),1)];
        if scanwhilemove
             workspace.indexedobsticles=remove_self_scanning(workspace.indexedobsticles,all_steps(step,:),1);
        end
        if rand>0.7            
            insidepoints = unique(insidepoints ,'rows');        
        end
    end
    %if we are asked to remove unknown point by moving through space
    robot_maxreach.pointcarvedout=unique([robot_maxreach.pointcarvedout;insidepoints],'rows');
      
    if remv_unkn_in_mv
       workspace.knowncoords=unique([workspace.knowncoords;insidepoints],'rows');
    end

    %set overall Q to be the last step in path
    Q=all_steps(end,:);
end