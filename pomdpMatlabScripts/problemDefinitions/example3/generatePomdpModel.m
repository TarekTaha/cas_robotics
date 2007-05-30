function generatePomdpModel(filename)
% This function generated the POMDP model according to Tony Cassandra's
% syntaxt. You have to specify the actions, observations and the
% uncertainty in their readings.

fid = fopen(filename, 'wb');
global mapTopology;
discount = 0.95;
actions = {'North','South','East','West','Stop'};
observations = {'Up','Down','Right','Left','Nothing'};
% Percentage Uncertainty in the actions
actionsUncertainty = [10,10,20,20,10];     
%actionsUncertainty = [0,0,0,0,0];     
% Percentage Uncertainty in the Observations
%observationsUncertainty = [10,20,15,10,15];
observationsUncertainty = [0,0,0,0,0];

% Construct the Graph network by defining the number of states and the
% translation inbetween the states. This will be used to build the
% translational probobilities with added uncertainties.

% define the destinations 
destinations = {1,6,26,30};
numSpacialStates = 49;
% Here we are building a model that is a cross product between states
% representiong spaces in the environment and destinations that the user
% wants to go to.

mapTopology.nnodes = numSpacialStates;
mapTopology.network = zeros(mapTopology.nnodes,length(actions));


    mapTopology.network(1,:) = [0,2,0,0,1]; 
    mapTopology.network(2,:) = [1,3,48,0,2]; 
    mapTopology.network(3,:) = [2,4,0,0,3]; 
    mapTopology.network(4,:) = [3,0,7,5,4]; 
    mapTopology.network(5,:) = [0,6,4,0,5]; 
    mapTopology.network(6,:) = [5,0,0,0,6]; 
    mapTopology.network(7,:) = [0,8,9,4,7]; 
    mapTopology.network(8,:) = [7,0,0,0,8]; 
    mapTopology.network(9,:) = [0,10,11,7,9]; 
    mapTopology.network(10,:) = [9,0,0,0,10]; 
    mapTopology.network(11,:) = [49,0,12,9,11]; 
    mapTopology.network(12,:) = [0,13,14,11,12]; 
    mapTopology.network(13,:) = [12,0,0,0,13]; 
    mapTopology.network(14,:) = [47,0,15,12,14]; 
    mapTopology.network(15,:) = [0,16,17,14,15]; 
    mapTopology.network(16,:) = [15,0,0,0,16]; 
    mapTopology.network(17,:) = [45,18,19,15,17]; 
    mapTopology.network(18,:) = [17,0,0,0,18]; 
    mapTopology.network(19,:) = [0,20,21,17,19]; 
    mapTopology.network(20,:) = [19,0,0,0,20]; 
    mapTopology.network(21,:) = [42,0,22,19,21]; 
    mapTopology.network(22,:) = [0,23,24,21,22]; 
    mapTopology.network(23,:) = [22,0,0,0,23]; 
    mapTopology.network(24,:) = [27,0,25,22,24]; 
    mapTopology.network(25,:) = [26,0,0,24,25]; 
    mapTopology.network(26,:) = [0,25,0,0,26]; 
    mapTopology.network(27,:) = [28,24,0,0,27]; 
    mapTopology.network(28,:) = [29,27,0,40,28]; 
    mapTopology.network(29,:) = [30,28,31,0,29]; 
    mapTopology.network(30,:) = [0,29,0,0,30]; 
    mapTopology.network(31,:) = [32,0,0,29,31]; 
    mapTopology.network(32,:) = [33,31,34,0,32]; 
    mapTopology.network(33,:) = [0,32,0,0,33]; 
    mapTopology.network(34,:) = [35,36,37,32,34]; 
    mapTopology.network(35,:) = [0,34,0,0,35]; 
    mapTopology.network(36,:) = [34,0,0,0,36]; 
    mapTopology.network(37,:) = [38,39,0,34,37]; 
    mapTopology.network(38,:) = [0,37,0,0,38]; 
    mapTopology.network(39,:) = [37,0,0,0,39]; 
    mapTopology.network(40,:) = [41,42,28,43,40]; 
    mapTopology.network(41,:) = [0,40,0,0,41]; 
    mapTopology.network(42,:) = [40,21,0,0,42]; 
    mapTopology.network(43,:) = [44,45,40,46,43]; 
    mapTopology.network(44,:) = [0,43,0,0,44]; 
    mapTopology.network(45,:) = [43,17,0,0,45]; 
    mapTopology.network(46,:) = [0,47,43,48,46]; 
    mapTopology.network(47,:) = [46,14,0,0,47]; 
    mapTopology.network(48,:) = [0,49,46,2,48]; 
    mapTopology.network(49,:) = [48,11,0,0,49]; 


% Write to File the top comments and warnings
fprintf(fid,'# This POMDP Model is generated Using Tarek''s MATLAB Script');
fprintf(fid,'\n# This script is still experimental and bugs might appear');
fprintf(fid,'\n# Tarek Taha - Center of Autonomous Systems, UTS\n');
% Specify Essential Parameters
fprintf(fid,'\ndiscount: %f',discount);
fprintf(fid,'\nvalues: reward');
fprintf(fid,'\n\nstates: ');
for i=1:mapTopology.nnodes
    fprintf(fid,'s%d ',i);
end

fprintf(fid,'\n\nactions:');
for i=1:length(actions)
    fprintf(fid,' %s',actions{i});
end
fprintf(fid,'\n\nobservations:');
for i=1:length(observations)
    fprintf(fid,' %s',observations{i});
end
startBelief = 1.0/mapTopology.nnodes;
fprintf(fid,'\nstart:');
for i=1:mapTopology.nnodes
    fprintf(fid,' %f',startBelief);
end

% Build the Translation probabilities
fprintf(fid,'\n\n# T: <action> : <start-state> : <end-state> %%f');
fprintf(fid,'\nT: * : * : * 0.0');
for i=1:length(actions)
    if length(find(actionsUncertainty)) ~=0
        actionProb = (100 - actionsUncertainty(i))/100;
    else
        actionProb = 1;
    end
    for j=1:mapTopology.nnodes
        % Find the number of reachable nodes (connected)
        n = length(find(mapTopology.network(j,:)));  
        % Uncertainty in the final state given this action is divided
        % equally to the connected neighbouring nodes
        if length(find(actionsUncertainty)) ~=0
            actionNoise = actionsUncertainty(i)/(n-1)/100;
        else
            actionNoise = 0;
        end
        % Undefined action to this state
        if (mapTopology.network(j,i)==0) 
            % Stay in place ? or shell i add noise to it ?
            fprintf(fid,'\nT: %s : s%d : s%d %f',actions{i},j,j,1.00);
            continue;
        end
        if floor(j/numSpacialStates) == 0 
            dest = 0;
        elseif mod(j,numSpacialStates) == 0
            dest  = floor(j/numSpacialStates) - 1;
        else
            dest  = floor(j/numSpacialStates) ;
        end    
        
        for s=1:length(actions)
            if (i==s) % The main Action
                fprintf(fid,'\nT: %s : s%d : s%d %f',actions{i},j,dest*numSpacialStates+mapTopology.network(j,s),actionProb);
            elseif(mapTopology.network(j,s)~=0 && length(find(actionsUncertainty)) ~=0)
                fprintf(fid,'\nT: %s : s%d : s%d %f',actions{i},j,dest*numSpacialStates+mapTopology.network(j,s),actionNoise);
            end
        end
        %fprintf(fid,'\n');
    end
    fprintf(fid,'\n#############################################');
end

% Build Observation Probabilities
fprintf(fid,'\n\n# O : <action> : <end-state> : <observation> %%f');
fprintf(fid,'\nO: * : * : * 0.0');
for i=1:mapTopology.nnodes
    % Find the number of reachable nodes (connected)
    n = length(find(mapTopology.network(i,:)));   
    if length(find(observationsUncertainty)) ~=0
        observationProb = 1/n;
    else
        observationProb = 1;
    end
    %Check the connections
    for s=1:length(actions)
        if (mapTopology.network(i,s)~=0) % The main Action
            fprintf(fid,'\nO: * : s%d : %s %f',i,observations{s},observationProb);
             if length(find(observationsUncertainty)) == 0
                 break;
             end
        end
    end
    fprintf(fid,'\n');    
end

% Build Reward Function
fprintf(fid,'\n\n# R: <action> : <start-state> : <end-state> : <observation> %%f');
fprintf(fid,'\nR: * : * : * : * 0.0');
for i=1:mapTopology.nnodes
    for s=1:length(actions)
        if (mapTopology.network(i,s)==0) % The main Action
            fprintf(fid,'\nR: %s : * : s%d : * -10',actions{s},i);
        end
    end
    fprintf(fid,'\n');
end
for i=1:length(destinations)
    fprintf(fid,'\nR: Stop : s%d : s%d : * 10',destinations{i},destinations{i});
    fprintf(fid,'\nR: *    : * : s%d : * 100',destinations{i});
end

fclose(fid);
end
