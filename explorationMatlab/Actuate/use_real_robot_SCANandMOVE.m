%% use_real_robot_SCANandMOVE
%
% *Description:*  This is called to move the sensor and is a way to
% modulate the scanning routine. Send the set of movements required while scanning  Q 
% must be globally defined.

%% Function Call
%
% *Inputs:* 
%
% _deg2scan_ (double) degrees - the number of degrees to tilt the scan
% through 6th joint
%OR
% _all_steps_ (double,many*6) degrees - the movements in rads
%
% *Returns:* NULL

function use_real_robot_SCANandMOVE(vargin)

%% Variables:  Declarations and checks
global G_scan robot_maxreach;
G_scan.PoseData=[];G_scan.RangeData=[];G_scan.IntensityData=[];G_scan.PointData=[];

hCOM=getappdata(gcf,'hCOM');

if nargin<1
    error('Need to pass in either the deg to scan with joint 6 or all_steps');
else
    if size(vargin,2)==1
        rad2scan=pi/180*vargin;
        tempQ=Q*180/pi;
        display([' Scanning through: ', num2str(rad2scan*180/pi)]);
        all_steps=[];
    else
        all_steps=vargin;
        if isempty(find(abs((all_steps(1,:)-all_steps(end,:)))>robot_maxreach.minjointres,1))
            %since we are already at the destination
            return
        end
            
%         display('Doing a moving Scan');
    end
end
    

%% Start Scanning/robot communication
hCOM.Laser.Type='RangeScan';
hCOM.Laser.TiltSpeed=robot_maxreach.scan_speed;

%hCOM.Laser.Mode='RangeOnly';
hCOM.Laser.Mode='RangeAndAveragedIntensity';        

%tilt q_5 in the negative direction so laser is safe
%(might be confusing)
hCOM.Laser.Start(-1);

%if we are just moving joint 6 then determine some steps
if isempty(all_steps)
    %steps to go through for joint 6
    numofsteps=20;
    all_steps=ones(numofsteps,1)*Q;
    all_steps(:,6)=[all_steps(:,6):rad2scan/(numofsteps-1):all_steps(:,6)+rad2scan]';  
end

%now we move through the desired steps
use_real_robot_MOVE(all_steps);

hCOM.Laser.Stop;   

