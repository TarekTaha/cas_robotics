%% use_real_robot_SCAN_q6
%
% *Description:*  This is called to do the actual scan and is a way to
% modulate the scanning routine. Simply input the degrees that are to be
% scanned Q must be globally defined, scan width should be globally defined
% This is the same as use_real_robot_SCAN but now the 6th joint will be
% moving

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

function use_real_robot_SCAN_q6(vargin)

%% Variables:  Declarations and checks
%THIS CLEARS GLOBAL VARS USED IN SCANS SO THEY DON'T BUILD UP
display('Clearing all global scan variables before starting a new scan');
clear global PoseData RangeData IntensityData PointData;

global scan Q r PointData IntensityData RangeData robot_maxreach robmap_h;

%make sure their is a surface map object
if isempty(robmap_h)
    display('Surface Map object has been deleted previously so I am recreating it')
    robmap_h=actxserver('EyeInHand.SurfaceMap');
    robmap_h.registerevent(@myhandler);
end

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
        display('Doing a moving Scan');
    end
end
    
%update the actual scanning origin
tr=fkine(r,Q);
scan.origin=tr(1:3,4)';


%% Start Scanning/robot communication
%give it an initial pose for base position)
robscan_h=robmap_h.ScannerCommand(eye(4));
robscan_h.TraceTo(['C:\data\', datestr(clock, 30), '_']);

robscan_h.Type='RangeScan';
robscan_h.TiltSpeed=robot_maxreach.scan_speed;

%robscan_h.Mode='RangeOnly';
robscan_h.Mode='RangeAndAveragedIntensity';        

%this is 120' either side so 2* 120
robscan_h.width=2*scan.theta*180/pi;

%display scan details
display(['Current Scan mode is: ',robscan_h.Mode,...
    '. Total width is: ',num2str(robscan_h.width)]);

%tilt through desired scan range in the negative direction so laser is safe
%(might be confusing)
robscan_h.Start(-1);

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

robscan_h.Stop;   
scan.PointData=PointData;
scan.IntensityData=IntensityData;
scan.RangeData=RangeData;

