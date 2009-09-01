%% use_real_robot_SCANandMOVE
%
% *Description:*  This is called to do the actual scan and is a way to
% modulate the scanning routine. Simply input the degrees that are to be
% scanned Q must be globally defined, scan width should be globally defined
% This will move while scanning and collect data like normal

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
%THIS CLEARS GLOBAL VARS USED IN SCANS SO THEY DON'T BUILD UP
% display('Clearing all global scan variables before starting a new scan');
clear global PoseData RangeData IntensityData PointData;

global scan Q r PointData IntensityData RangeData robot_maxreach;

hCOM=getappdata(gcf,'hCOM');

if size(Q,2)~=6
    error('Q - Joints have not been defined properly, should be a global');
end

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
    
%update the actual scanning origin
tr=fkine(r,Q);
scan.origin=tr(1:3,4)';


%% Start Scanning/robot communication
%give it an initial pose for base position)
hCOM.Laser.AddObserver(hCOM.Surface);

hCOM.Laser.Type='RangeScan';
hCOM.Laser.TiltSpeed=robot_maxreach.scan_speed;

%hCOM.Laser.Mode='RangeOnly';
hCOM.Laser.Mode='RangeAndAveragedIntensity';        

%this is 120' either side so 2* 120
hCOM.Laser.width=2*scan.theta*180/pi;

%display scan details
% display(['Current Scan mode is: ',hCOM.Laser.Mode,...
%     '. Total width is: ',num2str(hCOM.Laser.width)]);

%tilt through desired scan range in the negative direction so laser is safe
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
Q=all_steps(end,:);

hCOM.Laser.Stop;   
scan.PointData=PointData;
scan.IntensityData=IntensityData;
scan.RangeData=RangeData;

