%Function takes a set of target point and returns a solution set that
%covers all the tiles. Designed for RTA demonstation, does not handle 'do
%not go' space. Path decision based on robot joint cost and distances
%between targets.

function spraypoints = pathPlanner(plane)
%setupoptimisationV2
%Variable declaration
    global r mew;
    r = rob_object;
    mew = 0.04; %Tile diameter
    spray = zeros(1,length(plane)); %Array to mark which tile has been sprayed and in which order
    tile_distance = zeros(2, length(plane)); %array to store distance and corresponding tempplane array slot number
    totalDistance = 0; %Variable that accumunlates the distance travelled
    totaljointcost = [0 0 0 0 0 0]; %Variable that accumunlates the joint angle travelled
    spraypoints = struct('home_point',[0 0 0], 'equ', [0 0 0 0], 'Q',...
        [0 0 0 0 0 0], 'jointcost', [0 0 0 0 0 0]); %output structure containing home_points, equ and 
    pathPlan = spraypoints;
    counter = 1;
    spraycounter = 1;
    cycles = 0; %Number of cycles required to find path to every single tile

%Do a Pose Check on the plane set to determine if any points are
%unreachable. Discard unreachable points
    for i=1:length(plane)    
        if i > 1
            plane(i).Q = blasting_posesel(r, plane(i).home_point, plane(i).equ, plane(i-1).Q, true); 
        else
            plane(i).Q = blasting_posesel(r, plane(i).home_point, plane(i).equ, [0.5 0.5 0.5 0.5 0.5 0.5], true);   
        end
        tile = i
        PoseCheck(plane(i),1)
        if PoseCheck(plane(i),1) == 0
            spray(i) = 1
        end
    end
%keyboard    
%Find the leftmost tile to define as the arbitrary startpoint
%The first slot of struct 'spraypoints' will contain the determined startpoint
    spraypoints(1).home_point = plane(1).home_point;
    for i=1:length(plane)
        if spray(i) == 0
            if spraypoints(1).home_point(1) > plane(i).home_point(1)
                t = i; %slot number of tile in array
                spraypoints(1).home_point = plane(i).home_point;
                spraypoints(1).equ = plane(i).equ;
            end  
        end
    end

    spray(t) = 1; %Mark the startpoint as first element sprayed 
    currentslot = 1; %Internal indicator showing latest spraypoints struct slot created 

%Check how many cycles is required to assign all valid positions
    for i=1:length(spray)
        if spray(i) == 0
            cycles = cycles + 1;
        end
    end    
   
    for i1=1:cycles 
        counter = 1; %Resetting variables for the next iteration (internal calculation storage variable) 
        tile_distance = zeros(2, length(plane));%Resetting variables for the next (iteration calculation storage variable)
        clear potentialpoints; %Resetting internal variable
%Find the distances of the unsprayed tiles from the current tile
        for i=1:length(plane)%Output of this submodule is an 2xarray containing distances and the corresponding slot no.
            if spray(i) == 0 %Examine only unsprayed tiles
                %Calculate all the distances to unsprayed tiles from the
                %currentpoint
                d = sqrt((spraypoints(currentslot).home_point(1) - plane(i).home_point(1))^2 +...
                    (spraypoints(currentslot).home_point(2) - plane(i).home_point(2))^2 +...
                    (spraypoints(currentslot).home_point(3) - plane(i).home_point(3))^2);
                tile_distance(1,counter) = d;
                tile_distance(2,counter) = i;
                counter = counter + 1;
            end
        end
        
%Selection Sort the 3 closest distances and put them at the start of the tile_distance array
        for i=1:3 
            if tile_distance(1,i) ~= 0
                min = i;
                for j=i+1:length(tile_distance)
                    if tile_distance(1,j) ~= 0
                        if tile_distance(1,j)<tile_distance(1,min)
                            min = j;
                        end
                    end
                end
                temp = tile_distance(:,i);            
                tile_distance(:,i) = tile_distance(:,min);           
                tile_distance(:,min) = temp;           
            end
        end
tile_distance(2, 1:5)
 %Obtain the any necessary interval and joint cost data on the 3 closet tiles
        if tile_distance(2,3) ~= 0 %If there are more than 3 tiles under examination
            %Find out the joint cost from moving to each of the tiles
            potentialpoints(1,1).spraypoints = calculateJointCost(t, tile_distance(:,1), plane);        
            potentialpoints(1,2).spraypoints = calculateJointCost(t, tile_distance(:,2), plane);
            potentialpoints(1,3).spraypoints = calculateJointCost(t, tile_distance(:,3), plane);  
            potentialpointsRef = ones(1,length(potentialpoints)); %Interval variable for validity indication of the 3 potentialpoints
    %keyboard
            %Checking to see if path is valid, 0=invalid
            for i=1:length(potentialpoints)
                if pathCheck(potentialpoints(i).spraypoints) == 0 
                    potentialpointsRef(i)= 0;
                end           
            end
            potentialpointsRef
            potentialpoints(1).spraypoints(2).jointcost
            potentialpoints(2).spraypoints(2).jointcost
            potentialpoints(3).spraypoints(2).jointcost
            min = potentialpoints(3).spraypoints; %***NOTE this point is temporarily made as the deadlock fall back point
            leastcost = 3;
            %Where all three points are invalid, then this is the point it will
            %go to..currently assigned to potentialpoints(3)..need to change
            %this later
            %Checking to see which valid path has the lowest cost
            for i=1:length(potentialpoints)
    %keyboard
                if potentialpointsRef(i) ~= 0 & potentialpoints(i).spraypoints(2).jointcost < min(2).jointcost
                    min = potentialpoints(i).spraypoints;
                    leastcost = i;
                end
            end
        elseif tile_distance(2,2) ~= 0 %Scenario when only 2 tiles left             
            potentialpoints(1,1).spraypoints = calculateJointCost(t, tile_distance(:,1), plane);        
            potentialpoints(1,2).spraypoints = calculateJointCost(t, tile_distance(:,2), plane);
            potentialpointsRef = ones(1,length(potentialpoints));
            %Checking to see if path is valid
            for i=1:length(potentialpoints)
                if pathCheck(potentialpoints(i).spraypoints) == 0
                    potentialpointsRef(i) = 0;
                end
            end
            potentialpointsRef
            min = potentialpoints(2).spraypoints;
            leastcost = 2; 
            for i=1:length(potentialpoints)
    %keyboard
                if potentialpointsRef(i) ~= 0 & potentialpoints(i).spraypoints(2).jointcost < min(2).jointcost
                    min = potentialpoints(i).spraypoints;
                    leastcost = i;
                end
            end       
        else
            %Last tile to spray need to still implement pathCheck, do
            %later...assume it's ok to reach for the RTA demo test scenario
            min = calculateJointCost(t, tile_distance(:,1), plane);
            leastcost = 1;
        end
        leastcost
        
        nextTileMovement = tile_distance(2, leastcost)
        spray(tile_distance(2, leastcost)) = spraycounter + 1;
        spraycounter = spraycounter + 1;
        t = tile_distance(2, leastcost);  
        %Add the movement information from the chosen tile into the spraypoint
        %structure
        spraypoints(end) = min(1); %Overwrite the last entry of spraypoints with the first of min, because they are the same points but with more info
        for i = 2:size(min,2)
            spraypoints(end+1) = min(i);
        end    
        currentslot = size(spraypoints,2); %Indicates last slot of spraypoints
        totalDistance = totalDistance + tile_distance(1, leastcost);
        totaljointcost = totaljointcost + min(1).jointcost;       
    end
    totalDistance
    totaljointcost
end

function tempspraypoints = calculateJointCost(current, targetnext, plane) %inputs from tile_distance array, current is just the address ref
%keyboard
    global r mew;
%     mew = 0.04;
%     r = rob_object;

    intervals = round(targetnext(1,1)/(0.5*mew)); %Work out the required number of intemediate points, rounded off
    tempspraypoints = struct('home_point',[0 0 0], 'equ', [0 0 0 0], 'Q', [0 0 0 0 0 0], 'jointcost', [0 0 0 0 0 0]);
    tempspraypoints(1).home_point = plane(current).home_point; %Place the data of current point into slot 1 for the purpose of calculating joint cost
    tempspraypoints(1).equ = plane(current).equ;
    tempspraypoints(1).Q = plane(current).Q;
    if jointVariationCheck(plane(current), plane(targetnext(2,1))) == 1 %Ok to just move to the targetnext without intermediate points
        tempspraypoints(2).home_point = plane(targetnext(2,1)).home_point; %Place the data of current point into slot 1 for the purpose of calculating joint cost
        tempspraypoints(2).equ = plane(targetnext(2,1)).equ;
        tempspraypoints(2).Q = plane(targetnext(2,1)).Q;
    else %Place intermediate points at 1cm intervals..Modify this later dynamically make only required number of intermediate    
        for i=1:intervals %tempspraypoints stores points A to B with everything in between
            %r(t) = a + t(b-a)       
            tempspraypoints(i+1).home_point = plane(current).home_point + (i/intervals)*(plane(targetnext(2,1)).home_point - plane(current).home_point);       
            tempspraypoints(i+1).equ = plane(targetnext(2,1)).equ; %All intermediate points are on the same plane as targetnext point       
            tempspraypoints(i+1).Q = blasting_posesel(r, tempspraypoints(i+1).home_point, tempspraypoints(i+1).equ,tempspraypoints(i).Q, true);
            if i == intervals
                 tempspraypoints(i+2).home_point = plane(targetnext(2,1)).home_point; %Place targetnext at the end of the interval just in case
                 tempspraypoints(i+2).equ = plane(targetnext(2,1)).equ;
                 tempspraypoints(i+2).Q = plane(targetnext(2,1)).Q;
            end
        end
    end
    %Work out the joint cost for moving through those intervals, store in
    %first slot of tempspraypoints.jointcost
    for i=1:size(tempspraypoints,2)-1
        tempspraypoints(1).jointcost = abs(tempspraypoints(i+1).Q - tempspraypoints(i).Q) + tempspraypoints(1).jointcost;        
    end
    %Total of all 6 joints stored in second slot of
    %tempspraypoints.jointcost
    tempspraypoints(2).jointcost(1) = tempspraypoints(1).jointcost(1) + tempspraypoints(1).jointcost(2) +...
        tempspraypoints(1).jointcost(3) + tempspraypoints(1).jointcost(4) + tempspraypoints(1).jointcost(5) +...
        tempspraypoints(1).jointcost(6);     
end

%Check to see if the path set crosses any 'do not spray' areas and 
%Function returns 1 for valid path and 0 for invalid path
function booleanResult = pathCheck(tempspraypoints) %%Cut to only do PoseCheck for RTA demo
global plane;
booleanResult = 1;
%Don't need to check start and end points because already checked at start    
       
        if size(tempspraypoints,2) > 2 % Indicates there's intermediate points
            for j =2:size(tempspraypoints,2)-1 %Skip 1st and last slot               
                    %if sqrt((plane(i).home_point(1) - tempspraypoints(j).home_point(1))^2 +...
                        %(plane(i).home_point(2) - tempspraypoints(j).home_point(2))^2 +...
                        %(plane(i).home_point(3) - tempspraypoints(j).home_point(3))) < 0.13 | 
                 if PoseCheck(tempspraypoints(j),2) == 0 booleanResult=0; end   
            end            
        end
    %end
end

%Checks if the movement from point A to B is within the movement threshold
%that doesn't require intermediate points. 1 = valid movement
function boolean = jointVariationCheck(planeA, planeB)
    boolean = 1;
    if abs(planeA.Q(1) - planeB.Q(1)) > 0.4172 boolean=0; end
    if abs(planeA.Q(2) - planeB.Q(2)) > 0.0499 boolean=0; end
    if abs(planeA.Q(3) - planeB.Q(3)) > 0.0027 boolean=0; end
    if abs(planeA.Q(4) - planeB.Q(4)) > 0.0463 boolean=0; end
    if abs(planeA.Q(5) - planeB.Q(5)) > 0.0207 boolean=0; end
    if abs(planeA.Q(6) - planeB.Q(6)) > 0.1221 boolean=0; end
end