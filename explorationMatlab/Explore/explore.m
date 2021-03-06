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
global workspace G_scan bestviews r Q robot_maxreach

%% Selection of Scan Pose and direction

% this is either using a calculated NBV or choose our own
if true(useNBV)
%     db=bestviews(selection).scanorigin;
    G_scan.chosenview=bestviews(selection).chosenview;
    % Make it so that the last angle is 0
    newQ=[bestviews(selection).Q(1:5),0];
    %display('final angle has been change to 0 so we can get the largest scan')
else
    %Assume the first exploration step is from the (only) or at least first known point 
    if get(h.start_with_default_poses_checkbox,'value') && G_scan.tries<=size(robot_maxreach.default_Q,1)-1
        newQ=robot_maxreach.default_Q(G_scan.tries+1,:);       
        set(h.dialog_text,'String',strcat('Using default vals, now taking at exploration pose NO.',num2str(scan.tries+1)));
    else 
        display('You have asked to use a NBV and there are no more default poses so doing NBV_beta2');
        NBV_beta2();
        global bestviews;
%         db=bestviews(1).scanorigin;
        G_scan.chosenview=bestviews(1).chosenview;
        newQ=[bestviews(1).Q(1:5),0];
    end
end

%% Calculate what was known previously
previously_known=calcweight(0);

%% Save predicted information gain _for this timestep_ 
% Note:wont work when there is no selection
try workspace.predictedCoords=setdiff(workspace.unknowncoords,unique([workspace.knowncoords;workspace.indexedobsticles;bestviews(selection).expectedaddinfo],'rows'),'rows');end


%% Do the Scan _imaginary_
if (get(h.useRealRobot_checkbox,'Value')==0)
    if movetonewQ(h,newQ*180/pi)~=true
        error('No path could be found for the end position even if it is valid')
    end
    doscan(newQ);

%% Do the Scan _real_
else
    % Change newQ so if scanning on the downside (+ angles) -> start from max place and goes to the center, 
    % Alternately,it goes to +30', and then tilts to the max negative possible
    qlimits=r.qlim;
    minimum_alpha=qlimits(5,1)*0.9;
    %determine the maximum angle
    if newQ(3)>G_scan.alpha_limited_condition
        maximum_alpha=G_scan.alpha_limited;
    else 
        maximum_alpha=G_scan.alpha;
    end    
    %determine where to start and where to tilt through too
    if G_scan.chosenview(3)>0
        newQ(5)=min(maximum_alpha,newQ(5)+G_scan.alpha/2);
        tilt_scan_range=minimum_alpha-newQ(5);
    else
        newQ(5)=max(minimum_alpha,newQ(5)-G_scan.alpha/2);
        tilt_scan_range=maximum_alpha-newQ(5);
    end
    
    %try and move to newQ
    if movetonewQ(h,newQ*180/pi)~=true
        error('No path could be found for the end position even if it is valid')
    end
    
    %Tilt laser through determined tilt_scan_range
    use_real_robot_SCAN(tilt_scan_range*180/pi);    
    %this sort out the points that we have got from a scan
    organise_data();
    %if we want to do the classify and update voxels then we will do this here
    if get(h.doclassification_checkbox,'value')
        global G_scan;
        [ClassifiedData] = Block_Classifier(G_scan.PointData, G_scan.ntensityData); 
        update_ocstatus(ClassifiedData); 
        %try and save the current status, if user ctrl+c s out it dosen't matter
%         try AXBAMnCtesting(false);
%         catch
%             lasterr
%             keyboard
%         end
    end

end

    

%% Display results

% print results this exploration step details
try 
    tr=fkine(r,Q);
    display('. . . . . . . . . . . . . . . . . . . . . . . . ');
    display(strcat('The sensor origin is (',num2str([tr(1,4),tr(2,4),tr(3,4)]),'),TRY # :',num2str(G_scan.tries)));
    G_scan.ALLorigin=[G_scan.ALLorigin;[tr(1,4),tr(2,4),tr(3,4)]];
catch
    display('cant print out the sensor origin point');
end

% Calculate the WEIGHTED known/unknown/obstacle knowledge
knownweight=calcweight(0);
unknownweight=calcweight(0.5);
obstacleweight=calcweight(1);

% Print out the details of WEIGHTED known/unknown/obstacle knowledge
display(strcat('NEW Free Weight:',num2str(knownweight-previously_known),...
               ', Total Free Weight:',num2str(knownweight), ', Obstacles Weight:',num2str(obstacleweight),...
               ' Remain Unknown Weight:',num2str(unknownweight)));

% Set GUI to show size of known + obstacle knowledge out of total knowledge  
set(h.total_info_text,'String',num2str(size(workspace.knowncoords,1)+size(workspace.indexedobsticles,1)));
set(h.total_text,'String',num2str(size(workspace.unknowncoords,1)));
drawnow;

%increment the number of tries
G_scan.tries=G_scan.tries+1;