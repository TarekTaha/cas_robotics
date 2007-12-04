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
% display('Clearing all global scan variables before starting a new scan');
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

tempQ=Q*180/pi;

%update the actual scanning origin
tr=fkine(r,Q);
scan.origin=tr(1:3,4)';

%% Check view and make scan go in correct direction
if nargin==0
    if scan.chosenview(3)>0
        tilt_scan_range=round(-(Q(5)-scan.alpha)*180/pi);
    else
        tilt_scan_range=round(-(Q(5)+scan.alpha)*180/pi);
    end
else
    tilt_scan_range=deg2scan;
end

%uiwait(msgbox(strcat('The robot is currently at:', num2str(tempQ),' and is planning to scan through :',num2str(tilt_scan_range),'. Please get ready to push EMERGENCY STOP')));


%% Start Scanning/robot communication
attemptingscan=4; %used to set how many times we try before giving up

while attemptingscan
    %give it an initial pose for base position)
    robscan_h=robmap_h.ScannerCommand(eye(4));
    
    robscan_h.Type='TiltingRangeScan';
    robscan_h.TiltSpeed=robot_maxreach.scan_speed;
    
    %robscan_h.Mode='RangeOnly';
    robscan_h.Mode='RangeAndAveragedIntensity';        
    
    %this is 120' either side so 2* 120
    robscan_h.width=2*scan.theta*180/pi;

    %display scan details
%     display(strcat('Current Scan mode is: ',robscan_h.Mode,...
%         '. Total width is: ',num2str(robscan_h.width),...
%         '. Scanning through: ', num2str(tilt_scan_range)));

    %tilt through desired scan range in the negative direction so laser is safe (might be confusing)
    pause(0.05);
    robscan_h.Start(tilt_scan_range);
    
    pause(0.5);
    
    %if not started their must be a problem or if no data has started to be returned
    if(robscan_h.Started==0 && get(robscan_h,'Completed')==0)   
        release_scanner(robscan_h);

        if attemptingscan<=1 % it should break out here before getting through the while loop
            error('Scanner hasnt started yet there is a problem with the laser... Giving up');
        else
            display('Problem in this try - retrying');                           
            attemptingscan=attemptingscan-1;
            pause(0.2);
        end
    elseif isempty(PointData)
        display('Currently not data coming back: waiting for laser to complete before retrying');
        while get(robscan_h,'Completed')==0
            pause(0.5);display('.'); 
            %break out if we get some data back
            if ~isempty(PointData); break; end
        end
        %checks that there still really is no data coming back
        if ~isempty(PointData); 
            display('Data came back eventually');
            attemptingscan=0; 
        else
            release_scanner(robscan_h);
            error('No Data coming back ... Giving up');             
        end        
    else
        %it has started so we have no need to retry this breaks out of loop
        attemptingscan=0;
    end
end
    
%% Wait till complete
temp_counter=150; %simply keep on checking arbitary time if finished until actually finished
while get(robscan_h,'Completed')==0 || isempty(PointData)
    %display('waiting in scan');
    pause(0.5);
     if temp_counter<=0
         response=input(strcat('Completed=',num2str(get(robscan_h,'Completed')),...
                             ', PointData isempty=',num2str(isempty(PointData)),...
                             '. So we are waiting in scan: type (0) to quit, (1) for keyboard or anything else to continue\n'));
        if response==0
            release_scanner(robscan_h);
            error('something wrong with the laser scanning process');
        elseif response==1
            keyboard
        end
        temp_counter=10;
    else
        temp_counter=temp_counter-1;
    end
end

%need some way to save this to a variable that doesn't get deleted
scan.PointData=PointData;
scan.IntensityData=IntensityData;
scan.RangeData=RangeData;

release_scanner(robscan_h);

%% Check validity
if size(PointData,1)==0 && tilt_scan_range~=0
    error('Scanner did not return any data')
end

%% FUNCTION: Releases the scanner and reset the speed to correct move speed
function release_scanner(robscan_h)
global robot_maxreach
pause(0.1);    
robscan_h.TiltSpeed=robot_maxreach.move_speed;
robscan_h.Start(0);pause(0.1);
robscan_h.release;
pause(0.1);