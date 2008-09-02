%% use_real_robot_MOVE
%
% *Description:*  This can be called by anywhere and is a way to modulae to
% the moving routine. It starts a robot moving object 
% global Q - joints must be defined
% Note: Lots of magic numbers in this function but most are arbitory pauses
% for coms reasons or for error catching reasons, 
% the other numbers are soft motion checks for the arm but are not
% really needed since specific functions do checks during path planning

%% Function Call
%
% *Inputs:* 
%
% _all_steps_ (many*6 double) radians the steps that the arm must
% move through on its path
%
% *Returns:* NULL

function use_real_robot_MOVE(all_steps)

%% Variables
global Q robot_maxreach
if size(Q,2)~=6 && size(Q,2)~=7
    error('Q - Joints have not been defined properly, should be a global');
end

if nargin==0
    %No path has been passed in
    all_steps=[];
    tempQ=rad2deg(Q);   
else
  %get rid of crappy floating point unreachable joint commands (the 57 is
  %approx 180/pi so we don't get yucky floating points from pi 
  all_steps=round(all_steps/(robot_maxreach.minjointres/57))*(robot_maxreach.minjointres/57);
  tempQ=rad2deg(all_steps(end,:));
end

%can use the old way of updating the joint state
please_use_GETjsFunc=true;

%% Check that no steps exceed soft limits
if ~joint_softlimit_check(all_steps)
    display('there is a possible bad position in the joint steps, this shouldnt be planned, giving you control')
    keyboard
end
    
%% Setup robot movement object
rob_h=actxserver('EyeInHand.DensoCommand');
% rob_h2=actxserver('EyeInHand.DensoState');
%make sure comms up are up to speed
% waitcounter=0;
% while size(Q,2)~=6
%     pause(0.2);
%     if waitcounter>25 %5 secs
%         error('There is some problem with controller:m , make sure it is on, in auto mode, motor is on and program is running')
%     end    
% end
%check joint state and can continually check this to see that it has got to the correct position
% display(strcat('Current Joint State is:',num2str(rob_h2.JointStatee),' and the desired is:', num2str(tempQ)));

%% Check if all joints are the same as what we want them to be within 1degree
if ~isempty(find(round(tempQ-Q), 1))
    %Set the speed
    rob_h.Type='SetSpeed';
    rob_h.Params=[robot_maxreach.move_speed,0];

%% Move to an absolute joint reference if there is no path, otherwise
    if size(all_steps,1)<=1
        rob_h.Type='MoveJointAbs';
        % NOTE THE 6th JOINT IS OVERRIDEN TO 0 EARLIER in NBV so that it is perpendicular to the angle of rotations of joint 5 throughout the scan
        rob_h.Params=tempQ;
        display(strcat('The robot is currently at:', num2str(Q),' and is planning to move to:',num2str(rob_h.Params),'. Note 6th Joint may have been changed to 0 if using NBV. Please get ready to push EMERGENCY STOP'));
        % Additional Soft motion check - shouldn't be needed
        if ~joint_softlimit_check(Q) || ~joint_softlimit_check(tempQ)
            uiwait(msgbox('Failed softlimit check, make sure this point is within limitsD'));
            rob_h.release;
            error('Dont use this set of angles for joint 2 and 3');
        end
        rob_h.Start;
        % want it to wait until it has finished getting to the next posiiotn before scanning
        rob_h.WaitUntilCompleted(30,0);
        % simply release the object
        rob_h.release;

%% Drive the arm through the sequence of poses in all_steps
    else
        rob_h.Type='DriveJointAbs';
        for pathpnt=1:size(all_steps,1)
            current_step_DEG=all_steps(pathpnt,:)*180/pi;
            rob_h.Params=current_step_DEG;            
            if current_step_DEG(2)<-60
                % Additional Soft motion check - shouldn't be needed
                if (current_step_DEG(3)<-7/3*current_step_DEG(2)-130)
                    uiwait(msgbox('Note joint 2 is less than -60 and this could cause a problem with motion space; NOT CURRENTLY ALLOWED'));
                    rob_h.release;
                    error('Dont use this angle for joint 2 with the angle for joint3');
                end
            end
            %display(strcat('step No:',num2str(pathpnt),', Stepping to (in DEG):',num2str(current_step_DEG)));
            pause(0.1);rob_h.Start;pause(0.1);
            
            waitcoutner=0;           
            %our joint state here
            oldQ=Q;
            %check we are where we want to be            
            while max(abs((rad2deg(Q)-current_step_DEG)))>robot_maxreach.realMovementAngleInc
                pause(0.15);
                waitcoutner=waitcoutner+1;
                if waitcoutner==75 && ~isempty(find(abs(oldQ-Q)>0,1)) %about 15 seconds                    
                    button = questdlg('Did you press the emergency stop?','Running Slow');
                    if strcmp(button,'Yes')
                        releaserobot(rob_h)
                        error('emergency stop pressed - exiting');                    
                    else
                        %try alternate method of getting joint state
                        use_real_robot_GETJs()
                        rob_h.Params=current_step_DEG;
                        rob_h.Start;
                    end
                elseif waitcoutner>150 %about 30 seconds we have reissued the command and still no action
                    if ~isempty(find(abs(oldQ-Q)>robot_maxreach.minjointres,1))
                      keyboard
                      releaserobot(rob_h)
                      error('Problem issuing commands to drive the robot');                    
                    else
                      display('Although it is taking a long time, we still seem to moving');keyboard
                      oldQ=Q;
                    end
                end
                %if we need to update the old way
                if please_use_GETjsFunc;use_real_robot_GETJs();end
            end
        end
        %% Release movement object
        releaserobot(rob_h)
    end    
end

%% FUNCTION release the robot object
function releaserobot(rob_h)

    pause(0.1);
    rob_h.Type='StopDriveJointAbs';
    rob_h.Start;
    rob_h.WaitUntilCompleted(2,0)
    rob_h.release;
