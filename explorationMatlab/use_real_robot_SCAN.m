%% use_real_robot_SCAN
%
% *Description:*  This is called to tilt the sensor and is a way to
% modulate the scanning routine. Simply input the degrees that are to be
% scanned Q must be globally defined.

%% Function Call
%
% *Inputs:* 
%
% _deg2scan_ (double) degrees - the number of degrees to tilt the scan
% through
%
% *Returns:* NULL

function use_real_robot_SCAN(deg2scan)

if nargin<2; end

%% Variables:  Declarations and checks
global robot_maxreach G_scan;
G_scan.PoseData=[];G_scan.RangeData=[];G_scan.IntensityData=[];G_scan.PointData=[];

hCOM=getappdata(gcf,'hCOM');

%resync the robot and laser clock (must happen before every scan)
hCOM.App.ResyncDensoClock();

%% Start Scanning/robot communication     
hCOM.Laser.Type='TiltingRangeScan';
hCOM.Laser.TiltSpeed=robot_maxreach.scan_speed;

%robscan_h.Mode='RangeOnly';
hCOM.Laser.Mode='RangeAndAveragedIntensity';        

%tilt q_5 with negative rotate so laser is safe (might be confusing)
hTask=hCOM.Laser.Start(deg2scan);
pause(0.5);

msg{1}={'Please wait Scanning...'};msg{2}={'Possibly Timing out...'};

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
  warning(['Sensing didnt finish in the allotted time of ',num2str(max_allowable_count*0.1),'secs']);
else
  release(hTask);
end
delete(hWaitbar);

temp=sqrt(hCOM.Surface.LastSurfaceVariance);
disp(sprintf('Standard deviation at surface = %.5f',temp))

ARM_SetSpeed(gcf, robot_maxreach.move_speed);

    