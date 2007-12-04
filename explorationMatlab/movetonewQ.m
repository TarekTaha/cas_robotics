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
% _handles_ (array double) GUI variables
%
% _newQ_(1*6 double) newQ in DEGREES
%
% _all_steps_ (many*6) path of all 6 joints in RADs
%
% *Returns:* 
%
% _pathfound_ (binary) if a path was found or not

function pathfound=movetonewQ(handles,newQ,all_steps)

%% Variables
global r Q guiglobal workspace robot_maxreach;

% check we have got passed a newQ otherwise use from the GUI
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


%change to rads
newQ=newQ*pi/180;

scanwhilemove=get(handles.scanwhilemove_checkbox,'value');

%% Check if we only want to go to the exact destination
if get(handles.exact_joints_only_checkbox,'Value')
    tryalternate=false;
else
    tryalternate=true;
end

%% Get the latest Q from the robot if we are using it
if 	get(handles.useRealRobot_checkbox,'Value')
    use_real_robot_GETJs();
end

%% Check if we are already at destination or very close (rounding error)
%check if we are already at the destination and return if we are
%eps is a very small value and if there is less than this angular distance
%we disregard this and say it is at the same place
if isempty(find(abs(Q-newQ)>eps, 1));
    %since we are already at the correct path
    set(handles.dialog_text,'String','Already at destination');drawnow;
    if get(handles.show_robot_checkbox,'Value');
        plotdenso(r, newQ, guiglobal.checkFF, guiglobal.plot_ellipse);
    end
    %since we are already there
    pathfound=1;
    return
end

%% Do path planning
% Provided we haven't passed a path in which is full and has a start at Q
% and finish at newQ, also that they are all numbers
if ~isempty(all_steps) && ...
        isempty(find(all_steps(1,1:3)-Q(1:3)>eps,1)) && ...
        isempty(find(all_steps(end,1:3)-newQ(1:3)>eps,1)) &&...
        isempty(find(isnan(all_steps),1))
    %check the path passed in for collisions
    pathfound=check_path_for_col(all_steps);
    if ~pathfound
        display('The passed in path is not valid - calculating another');
        try [pathfound,all_steps]=pathplanner_new(newQ,guiglobal.plotpath,tryalternate);end
    end     
else % no valid path has been passed
    try %set(handles.dialog_text,'String','Calculating Path......');drawnow;
    %     [pathfound,all_steps]=pathplanner(newQ,guiglobal.plotpath,tryalternate);
tic
        [pathfound,all_steps]=pathplanner_new(newQ,guiglobal.plotpath,tryalternate);
display(['Additional Path Planning time is: ',num2str(toc)]);
    catch; keyboard
    end
end

%if it is an error returned means that the end is not valid
if pathfound==-1
    set(handles.dialog_text,'String','Impossible Path - Collision avoided! Ignoring command!');
    error('It is not safe/possible to move here');
end

%% If there is a path then do the actual move(and plot) else do nothing
if pathfound && size(all_steps,1)>0
    %if animate has been selected it will go through each step of the path
    if get(handles.animate_move_checkbox,'Value')
        set(handles.dialog_text,'String','Path found - animating....');
%         demopath(all_steps);       
        demopath_new(all_steps);
    end
    %plot the end effector in place
    if get(handles.show_robot_checkbox,'Value');
        plotdenso(r, all_steps(end,:), guiglobal.checkFF, guiglobal.plot_ellipse);
    end
    set(handles.dialog_text,'String','Path found - animation updated');
    %If using real robot try and move it
    if get(handles.useRealRobot_checkbox,'Value')
        try if ~scanwhilemove
                use_real_robot_MOVE(all_steps); 
                set(handles.dialog_text,'String','Actual robot movement complete');
            else
% tic
                use_real_robot_SCANandMOVE(all_steps);
% toc
                %try but if no data because already at the end then don't worry
% tic
                try organise_data(); end;
% toc
                set(handles.dialog_text,'String','Scan N Move completed');
            end
            %save the successfull path incase we need to retrace our steps
            if (size(all_steps,1)>3 && ~isempty(find(abs(all_steps(1,1:4)-all_steps(end,1:4))>pi/180,1))) ||...
                    isempty(robot_maxreach.path)
                robot_maxreach.path(end+1).all_steps=all_steps;
            else %add the slight movement (scan move/adjust) onto the end of last path
                robot_maxreach.path(end).all_steps=[robot_maxreach.path(end).all_steps;all_steps];
            end
            
        catch set(handles.dialog_text,'String','Error: Did not complete movement - Emergency Stop Probably Hit');
            lasterr;
            error('Did not complete movement');
        end
    end
% tic
    %remove obstacle points that are recorded but were in the path    
    insidepoints=[];
    for step=1:size(all_steps,1)           
        insidepoints = [insidepoints;find_points_in_FF(workspace.unknowncoords(workspace.lev1unknown,:),all_steps(step,:),1)];
        if scanwhilemove
%             workspace.Nobsticlepoints=remove_self_scanning(workspace.Nobsticlepoints,all_steps(step,:),1);
             workspace.indexedobsticles=remove_self_scanning(workspace.indexedobsticles,all_steps(step,:),1);
        end
        if rand>0.7            
            insidepoints = unique(insidepoints ,'rows');        
        end
    end
    %if we are asked to remove unknown point by moving through space
    robot_maxreach.pointcarvedout=unique([robot_maxreach.pointcarvedout;insidepoints],'rows');
    
    %since later on we are using only the obstacle points so we need to
    %update this set
%     if scanwhilemove
%         workspace.indexedobsticles=unique(round(workspace.Nobsticlepoints/workspace.inc_size)*workspace.inc_size,'rows');
%         %update the obstacle points with only obstacles that weren't in any
%         workspace.indexedobsticles=setdiff(workspace.indexedobsticles,robot_maxreach.pointcarvedout,'rows');        
%     end
    
    if get(handles.remv_unkn_in_mv_checkbox,'value')
       workspace.knowncoords=unique([workspace.knowncoords;insidepoints],'rows');
    end
% toc

    %set overall Q to be the last step in path
    Q=all_steps(end,:);
else %no path found, update GUI accordingly
      
    if get(handles.show_robot_checkbox,'Value');
        plotdenso(r, Q, guiglobal.checkFF, guiglobal.plot_ellipse);
    end
    set(handles.dialog_text,'String','End valid but no path found');
end