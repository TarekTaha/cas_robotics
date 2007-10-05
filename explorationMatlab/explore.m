%% explore
%
% *Description:*  This function alows: 
% * 1) Virtual world: for a selection of where to explore of uses a selected bestview that
% has been precalculated then works out gained knowledge and add it to the
% map
% * 2) Real World: Move the robot to desired NBV or chosen place then add
% points to memory,
% * _Note:_ All needs to be setup correctly including workspace, the scan
% parameters, robot (r) and Q and parameters robot_maxreach and the exGUI 

%% Function Call
%
% *Inputs:* 
%
% _h_ (struct double) handles of the GUI,
%
% _useNBV_ (bin) whether or not the best view is to be used
%
% _selection_ (int) if useNBV, which view to use
%
% *Returns:* NULL

function explore(h,useNBV,selection)

%% Variables
global workspace scan bestviews r Q robot_maxreach

%% Selection of Scan Pose and direction

% this is either using a calculated NBV or choose our own
if true(useNBV)
    scan.origin=bestviews(selection).scanorigin;
    scan.chosenview=bestviews(selection).chosenview;
    % Make it so that the last angle is 0
    newQ=[bestviews(selection).Q(1:5),0];
    %display('final angle has been change to 0 so we can get the largest scan')
else
    %Assume the first scan is from the (only) or at least first known point 
    if get(h.start_with_default_poses_checkbox,'value') && scan.tries<=size(robot_maxreach.default_Q,1)-1
        newQ=robot_maxreach.default_Q(scan.tries+1,:);       
        set(h.dialog_text,'String',strcat('Using default vals, now taking scan NO.',num2str(scan.tries+1)));
        tr=fkine(r,Q);
        scan.origin=[tr(1,4),tr(2,4),tr(3,4)];
    else 
        error('You have asked for not a NBV scan and there are no more default poses');
    end
end

%% Calculate what was known previously
previously_known=calknownweight();

%% Do the Scan _imaginary_
if (get(h.useRealRobot_checkbox,'Value')==0)
    if movetonewQ(h,newQ*180/pi)~=true
        error('No path could be found for the end position even if it is valid')
    end
    doscan(newQ);

%% Do the Scan _real_
else
    % Change newQ so if scanning on the downside (+ angles) -> start from max place and goes to the center, 
    % Alternately,it goes to +30', and then scan to the max negative possible
    qlimits=r.qlim;
    minimum_alpha=qlimits(5,1)*0.9;
    %determine the maximum angle
    if newQ(3)>scan.alpha_limited_condition
        maximum_alpha=scan.alpha_limited;
    else 
        maximum_alpha=scan.alpha;
    end    
    %determine where to start and where to tilt through too
    if scan.chosenview(3)>0
        newQ(5)=min(maximum_alpha,newQ(5)+scan.alpha/2);
        tilt_scan_range=minimum_alpha-newQ(5);
    else
        newQ(5)=max(minimum_alpha,newQ(5)-scan.alpha/2);
        tilt_scan_range=maximum_alpha-newQ(5);
    end
    
    %try and move to newQ
    if movetonewQ(h,newQ*180/pi)~=true
        error('No path could be found for the end position even if it is valid')
    end
    
    %take a scan through determined tilt_scan_range
    use_real_robot_SCAN(tilt_scan_range*180/pi);
        
    %this sort out the points that we have got from a scan
    organise_data(tilt_scan_range);
end

%% Display results

% print results this exploration step details
display(strcat('The scan origin is (',num2str(scan.origin),'), [with 0=+ & 1=-] pose is (abc)  (',num2str(scan.chosenview),...
               '), and this is try:',num2str(scan.tries)));

scan.ALLorigin=[scan.ALLorigin;scan.origin];

% Calculate the WEIGHTED known/unknown/obstacle knowledge
unknownweight=calunknownweight();
knownweight=calknownweight();
obstacleweight=calobstacleweight();

% Print out the details of WEIGHTED known/unknown/obstacle knowledge
display(strcat('New freespace knowledge:',num2str(knownweight-previously_known),...
               ', Weighted Total Known Free:',num2str(knownweight), ', Weighted Obsticles:',num2str(obstacleweight),...
               ' Remaining Weighted Unknown withing workspace:',num2str(unknownweight)));

% Set GUI to show size of known + obstacle knowledge out of total knowledge  
set(h.total_info_text,'String',num2str(size(workspace.knowncoords,1)+size(workspace.indexedobsticles,1)));
set(h.total_text,'String',num2str(size(workspace.unknowncoords,1)));
drawnow;

%increment the number of tries
scan.tries=scan.tries+1;