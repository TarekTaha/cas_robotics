%% use_real_robot_SCAN
%
% *Description:*  This is called to do the actual scan and is a way to
% modulate the scanning routine. Simply input the degrees that are to be
% scanned Q must be globally defined, scan width should be globally defined

%% Function Call
%
% *Inputs:* 
%
% _deg2scan_ (double) degrees - the number of degrees to tilt the scan
% through
%
% *Returns:* NULL

function use_real_robot_SCAN(deg2scan)

%% Variables:  Declarations and checks
%THIS CLEARS GLOBAL VARS USED IN SCANS SO THEY DON'T BUILD UP
clear global PoseData RangeData IntensityData PointData;

global robot_maxreach PointData IntensityData RangeData scan;

hCOM=getappdata(gcf,'hCOM');

%resync the robot and laser clock (must happen before every scan)
hCOM.App.ResyncDensoClock();

%% Start Scanning/robot communication     
hCOM.Laser.Type='TiltingRangeScan';
hCOM.Laser.TiltSpeed=robot_maxreach.scan_speed;

%robscan_h.Mode='RangeOnly';
hCOM.Laser.Mode='RangeAndAveragedIntensity';        

%tilt through desired scan range in the negative direction so laser is safe (might be confusing)
hTask=hCOM.Laser.Start(deg2scan);
msg{1}={'Please wait Scanning...'};msg{2}={'Timing out...'};

hWaitbar = waitbar(0, 'Please wait ...', 'name', 'Scanning');
stepCount = 0;
max_allowable_count=1000;
while hTask.CompletedPortion < 1 && stepCount < max_allowable_count
  pause(0.1);
  [value,index]=max([hTask.CompletedPortion,stepCount/max_allowable_count]);
  waitbar(value, hWaitbar,msg{index});
  stepCount = stepCount + 1;
end
if stepCount==max_allowable_count
  warning(['The scan didnt finish in the allotted time of ',num2str(max_allowable_count*0.1),'secs']);
else
  release(hTask);
end
delete(hWaitbar);

%only used in my surface inspection method (poseclassunknown.m)
scan.PointData=PointData;
scan.IntensityData=IntensityData;
scan.RangeData=RangeData;

    