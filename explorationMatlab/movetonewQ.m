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
% _newQ_(1*6 double) newQ in degrees
%
% *Returns:* 
%
% _pathfound_ (binary) if a path was found or not

function pathfound=movetonewQ(handles,newQ)

%% Variables
global r Q guiglobal;

% check we have got passed a newQ otherwise use from the GUI
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

%change to rads
newQ=newQ*pi/180;

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
if isempty(find(abs(Q-(newQ*pi/180))>eps, 1));
    %since we are already at the correct path
    set(handles.dialog_text,'String','Already at destination');drawnow;
    if get(handles.show_robot_checkbox,'Value');
        plotdenso(r, newQ, guiglobal.checkFF, guiglobal.plot_ellipse);
    end
    return
end

%% Do path planning
try %set(handles.dialog_text,'String','Calculating Path......');drawnow;
    [pathfound,all_steps]=pathplanner(newQ,guiglobal.plotpath,tryalternate);
catch; keyboard
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
        demopath(all_steps);       
    end
    %plot the end effector in place
    if get(handles.show_robot_checkbox,'Value');
        plotdenso(r, all_steps(end,:), guiglobal.checkFF, guiglobal.plot_ellipse);
    end
    set(handles.dialog_text,'String','Path found - animation updated');
    %If using real robot try and move it
    if get(handles.useRealRobot_checkbox,'Value')
        try use_real_robot_MOVE(all_steps); 
            set(handles.dialog_text,'String','Actual robot movement complete');
        catch set(handles.dialog_text,'String','Error: Did not complete movement - Emergency Stop Probably Hit');
            error('Did not complete movement');
        end
    end
    %set overall Q to be the last step in path
    Q=all_steps(end,:);
else %no path found, update GUI accordingly
    if get(handles.show_robot_checkbox,'Value');
        plotdenso(r, Q, guiglobal.checkFF, guiglobal.plot_ellipse);
    end
    set(handles.dialog_text,'String','End valid but no path found');
end