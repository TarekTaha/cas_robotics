%Includes surface making simple set to create the plane, the only piece of
%information that this module requires is the points representing the
%surface
function output = geneticPlannerHybrid(points, iteration, plane, handles, show, vidCap) %provide the mesh points
mew = 0.04;
global r pose
tic
%******************************************************************
%load tempdata %Stub for surface making and pre GA pose selection
%******************************************************************
%Pre GA pose selection
%Note: variable Plane can't be reused if there are invalid target point in
%Need to rebuild a new target point list containing only valid ones
%variable = 'planeValid'
set(handles.text5, 'String', 'Pre GA Pose Selection');
for i = 1:length(plane)
%Create the cities XYZ list for the GA and check to see if the cities can return a
%pose, if no pose is available then discard the tile from planning. 
    if i == 1
        %[jointConfig,valid,tempdist]= blasting_posesel040908(r, plane(i).home_point, plane(i).equ, [0 0 0 0 0 0], false); 
        [jointConfig,valid,tempdist]= blasting_posesel(r, plane(i).home_point, plane(i).equ, [0 0 0 0 0 0], false); 
    else
        [jointConfig,valid,tempdist] = blasting_posesel(r, plane(i).home_point, plane(i).equ, jointConfig, false); 
    end
    if valid == true %Valid point, has a pose
        if ~exist('cities')
            cities(1,:) = plane(i).home_point;
            pose(1).Q = jointConfig;
            plane(1).validPose = true;
            planeValid(1) = plane(i);
        else
            cities(end+1,:) = plane(i).home_point;
            pose(end+1).Q = jointConfig;
            plane(i).validPose = true;
            planeValid(end+1) = plane(i);
        end 
    else
        plane(i).validPose = false; %Mark the target point as no good, not put through to GA
        %keyboard
    end        
end
%keyboard

set(handles.text5, 'String', 'Pre GA Pose Selection Complete');
%******************************************************************
%keyboard
%******************************************************************
display('GA start')
set(handles.text5, 'String', 'GA start');
[sorted_cities, best_route, distance, pop] = tsp_gaHybridVid033008(cities, 'popsize', 60, 'mrate', 0.75, 'numiter', iteration);
%Perform pose selection on the population list
%toc
display('GA complete')
set(handles.text5, 'String', 'GA Complete');
%******************************************************************

%******************************************************************
% Run pose selection on the best route to update the variable pose.q into pose2.q with
% better joint configs
% Variable 'pose2' is the post GA joint config list
set(handles.text5, 'String', 'Starting Post GA Pose Selection');
display('Starting Post GA Pose Selection')
for i=1:length(best_route)
    if i == 1
        [jointConfig,valid,tempdist]= blasting_posesel(r, planeValid(best_route(i)).home_point, planeValid(best_route(i)).equ, [0 0 0 0 0 0], false); 
    else
        [jointConfig,valid,tempdist] = blasting_posesel(r, planeValid(best_route(i)).home_point, planeValid(best_route(i)).equ, jointConfig, false); 
    end
    if valid == true %Valid point, has a pose
        pose2(best_route(i)).Q = jointConfig;
    else
        pose2(best_route(i)).Q = pose(best_route(i)).Q;
        %keyboard
    end
end
toc
display('Post Pose Selection Complete')
set(handles.text5, 'String', 'Post Pose Selection Complete');
%******************************************************************
%load tempData2
%******************************************************************
% Quality Parameter calculation

%Vaid target point tiles
if show == true
    sprayPath040708(plane,0.04,0.01,'r')
end
% Cumulative Distance comparision between original and GA
for i =1:length(best_route)-1
    if i == 1
        cumulativeDistanceOriginal(i)= sqrt((planeValid(i).home_point(1) - planeValid(i+1).home_point(1))^2 +...
                                (planeValid(i).home_point(2) - planeValid(i+1).home_point(2))^2 +...
                                (planeValid(i).home_point(3) - planeValid(i+1).home_point(3))^2);

        cumulativeDistanceGA(i)= sqrt((planeValid(best_route(i)).home_point(1) - planeValid(best_route(i+1)).home_point(1))^2 +...
                                (planeValid(best_route(i)).home_point(2) - planeValid(best_route(i+1)).home_point(2))^2 +...
                                (planeValid(best_route(i)).home_point(3) - planeValid(best_route(i+1)).home_point(3))^2);
    else
        cumulativeDistanceOriginal(i)= sqrt((planeValid(i).home_point(1) - planeValid(i+1).home_point(1))^2 +...
                            (planeValid(i).home_point(2) - planeValid(i+1).home_point(2))^2 +...
                            (planeValid(i).home_point(3) - planeValid(i+1).home_point(3))^2) + cumulativeDistanceOriginal(i-1) ;

        cumulativeDistanceGA(i)= sqrt((planeValid(best_route(i)).home_point(1) - planeValid(best_route(i+1)).home_point(1))^2 +...
                            (planeValid(best_route(i)).home_point(2) - planeValid(best_route(i+1)).home_point(2))^2 +...
                            (planeValid(best_route(i)).home_point(3) - planeValid(best_route(i+1)).home_point(3))^2) + cumulativeDistanceGA(i-1);
    end
end
if show == true 
    figure(2);title('Cumulative Distance');plot(cumulativeDistanceOriginal(1:length(cumulativeDistanceOriginal)), 'r', 'LineWidth', 2);hold on; 
    plot(cumulativeDistanceGA(1:length(cumulativeDistanceGA)), 'b', 'LineWidth', 2); hold off
end

%Cumulative Array of Joint Best Route and Realtime diagram
for i=1:size(best_route,2)
    jointSequence(i).Q = pose2(best_route(i)).Q; %Rearrange in terms of sequence
end
%if show == true figure(2);title('Joint Graph');jointGraph021208(jointSequence,jointSequence,jointSequence); end

for i=1:length(jointSequence)-1
    tempGA = abs(jointSequence(i).Q-jointSequence(i+1).Q);
    if i == 1
        jointCostGA(i) = tempGA(1)+tempGA(2)+tempGA(3)+tempGA(4)+tempGA(5);
    else
        jointCostGA(i) = jointCostGA(i-1) + tempGA(1)+tempGA(2)+tempGA(3)+tempGA(4)+tempGA(5);
    end    
    tempOriginal = abs(pose2(i).Q - pose2(i+1).Q);
    if i == 1
        jointCostOriginal(i) = tempOriginal(1)+tempOriginal(2)+tempOriginal(3)+tempOriginal(4)+tempOriginal(5);
    else
        jointCostOriginal(i) = jointCostOriginal(i-1) + tempOriginal(1)+tempOriginal(2)+tempOriginal(3)+tempOriginal(4)+tempOriginal(5);
    end    
end
if show == true 
    figure(3);title('Cumulative Joint');plot(jointCostGA(1:length(jointCostGA)), 'b', 'LineWidth', 2);hold on; 
    plot(jointCostOriginal(1:length(jointCostOriginal)), 'r', 'LineWidth', 2);hold off; 
end

%Coverage Calculation
for i=1:length(best_route)
    pathPlan(i).home_point = planeValid(best_route(i)).home_point;
end
if vidCap == true    
    [P0,baseline,excess]=coverageCalculation040908(pathPlan,pose2,points,'coverageGA',true,true);
    [P0,baseline,excess]=coverageCalculation040908(planeValid,pose,points,'coverageOriginal',true,true);
else
    if show == true
        %keyboard
        profile clear;profile on;
        [P0GA,baseline,excess]=coverageCalculation040908(pathPlan,pose2,points,'coverageGA',false,false);        
        profile off;profile viewer;
        %keyboard
        %[P0Original,baseline,excess]=coverageCalculation040908(planeValid,pose,points,'coverageOriginal',false,false);
        figure(4)
        subplot(1,2,1)
        %keyboard
        plot3(points(:,1),points(:,2),points(:,3),'b.','Markersize', 0.5)
        for j=0.02:0.02:1        
            covered=find(P0GA(:,4)<= j & P0GA(:,4) > j-0.02);        
            if ~isempty(covered)
                plothandle(int32(j/0.02))=plot3(P0GA(covered,1),P0GA(covered,2),P0GA(covered,3),'color',[1-j,1-j,1-j],'marker','.','linestyle','none');
                
            end
        end
        %keyboard
        subplot(1,2,2)
        for j=0.02:0.02:1        
        covered=find(P0Original(:,4)<= j & P0Original(:,4) > j-0.02);        
            if ~isempty(covered)
                plothandle(int32(j/0.02))=plot3(P0Original(covered,1),P0Original(covered,2),P0Original(covered,3),'color',[1-j,1-j,1-j],'marker','.','linestyle','none');
                %drawnow
            end
        end
    end
end

if show == true 
    figure(5);
    subplot(1,2,1)
    pathDiagram040708(pathPlan, points, 'b');
    subplot(1,2,2)
    pathDiagram040708(planeValid, points, 'r');
end
output.jointSequence = jointSequence;
output.pathPlan = pathPlan;
%output.P0 = P0;
save GA_data
%******************************************************************
