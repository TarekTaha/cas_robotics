%% get_robot_joint_state
%
% *Description:*  This function is for finding out the current joint state

%% Function Call
%
% *Inputs:*  NULL
% 
% *Returns:*  NULL

function use_real_robot_GETJs()

%% Variables
global Q

%% Establish Comms
% rob_h=actxserver('EyeInHand.DensoCommand');
rob_h=actxserver('EyeInHand.DensoQuery');
rob_h.Type = 'GetJointPos';

%make sure comms up are up to speed
waitcounter=0;

%% Get the Jointstate and save to global Q
% try while size(rob_h.JointState,2)~=6
try while size(rob_h.Value,2)~=6

        
        pause(0.2);
        if waitcounter>25 %5 secs
            error('exGUI - move function: You say you are trying to use a real robot, but I cant talk to it, I need its actaul current pose - make sure it is on, in auto mode, motor is on and program is running')
        end    
    end
%     Q=rob_h.JointState*pi/180;
    Q=rob_h.Value*pi/180;
    
catch;
    display('an error has occured when retrieving the jointstate');
end

rob_h.release;
