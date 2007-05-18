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
%actionsUncertainty = [10,10,20,20,10];    
actionsUncertainty = [0,0,0,0,0];
% Percentage Uncertainty in the Observations
%bservationsUncertainty = [10,10,10,10,15];
observationsUncertainty = [0,0,0,0,0];

% Construct the Graph network by defining the number of states and the
% translation inbetween the states. This will be used to build the
% translational probobilities with added uncertainties.

% define the destinations 
destinations = {'s1d1','s6d2','s26d3','s30d4'};
numSpacialStates = 49;
% Here we are building a model that is a cross product between states
% representiong spaces in the environment and destinations that the user
% wants to go to.

mapTopology.nnodes = numSpacialStates*length(destinations);
mapTopology.network = zeros(mapTopology.nnodes,length(actions));

for i=0:(length(destinations)-1)
    mapTopology.network(1+i*numSpacialStates,:) = [0,2,0,0,1]; 
    mapTopology.network(2+i*numSpacialStates,:) = [1,3,48,0,2]; 
    mapTopology.network(3+i*numSpacialStates,:) = [2,4,0,0,3]; 
    mapTopology.network(4+i*numSpacialStates,:) = [3,0,7,5,4]; 
    mapTopology.network(5+i*numSpacialStates,:) = [0,6,4,0,5]; 
    mapTopology.network(6+i*numSpacialStates,:) = [5,0,0,0,6]; 
    mapTopology.network(7+i*numSpacialStates,:) = [0,8,9,4,7]; 
    mapTopology.network(8+i*numSpacialStates,:) = [7,0,0,0,8]; 
    mapTopology.network(9+i*numSpacialStates,:) = [0,10,11,7,9]; 
    mapTopology.network(10+i*numSpacialStates,:) = [9,0,0,0,10]; 
    mapTopology.network(11+i*numSpacialStates,:) = [49,0,12,9,11]; 
    mapTopology.network(12+i*numSpacialStates,:) = [0,13,14,11,12]; 
    mapTopology.network(13+i*numSpacialStates,:) = [12,0,0,0,13]; 
    mapTopology.network(14+i*numSpacialStates,:) = [47,0,15,12,14]; 
    mapTopology.network(15+i*numSpacialStates,:) = [0,16,17,14,15]; 
    mapTopology.network(16+i*numSpacialStates,:) = [15,0,0,0,16]; 
    mapTopology.network(17+i*numSpacialStates,:) = [45,18,19,15,17]; 
    mapTopology.network(18+i*numSpacialStates,:) = [17,0,0,0,18]; 
    mapTopology.network(19+i*numSpacialStates,:) = [0,20,21,17,19]; 
    mapTopology.network(20+i*numSpacialStates,:) = [19,0,0,0,20]; 
    mapTopology.network(21+i*numSpacialStates,:) = [42,0,22,19,21]; 
    mapTopology.network(22+i*numSpacialStates,:) = [0,23,24,21,22]; 
    mapTopology.network(23+i*numSpacialStates,:) = [22,0,0,0,23]; 
    mapTopology.network(24+i*numSpacialStates,:) = [27,0,25,22,24]; 
    mapTopology.network(25+i*numSpacialStates,:) = [26,0,0,24,25]; 
    mapTopology.network(26+i*numSpacialStates,:) = [0,25,0,0,26]; 
    mapTopology.network(27+i*numSpacialStates,:) = [28,24,0,0,27]; 
    mapTopology.network(28+i*numSpacialStates,:) = [29,27,0,40,28]; 
    mapTopology.network(29+i*numSpacialStates,:) = [30,28,31,0,29]; 
    mapTopology.network(30+i*numSpacialStates,:) = [0,29,0,0,30]; 
    mapTopology.network(31+i*numSpacialStates,:) = [32,0,0,29,31]; 
    mapTopology.network(32+i*numSpacialStates,:) = [33,31,34,0,32]; 
    mapTopology.network(33+i*numSpacialStates,:) = [0,32,0,0,33]; 
    mapTopology.network(34+i*numSpacialStates,:) = [35,36,37,32,34]; 
    mapTopology.network(35+i*numSpacialStates,:) = [0,34,0,0,35]; 
    mapTopology.network(36+i*numSpacialStates,:) = [34,0,0,0,36]; 
    mapTopology.network(37+i*numSpacialStates,:) = [38,39,0,34,37]; 
    mapTopology.network(38+i*numSpacialStates,:) = [0,37,0,0,38]; 
    mapTopology.network(39+i*numSpacialStates,:) = [37,0,0,0,39]; 
    mapTopology.network(40+i*numSpacialStates,:) = [41,42,28,43,40]; 
    mapTopology.network(41+i*numSpacialStates,:) = [0,40,0,0,41]; 
    mapTopology.network(42+i*numSpacialStates,:) = [40,21,0,0,42]; 
    mapTopology.network(43+i*numSpacialStates,:) = [44,45,40,46,43]; 
    mapTopology.network(44+i*numSpacialStates,:) = [0,43,0,0,44]; 
    mapTopology.network(45+i*numSpacialStates,:) = [43,17,0,0,45]; 
    mapTopology.network(46+i*numSpacialStates,:) = [0,47,43,48,46]; 
    mapTopology.network(47+i*numSpacialStates,:) = [46,14,0,0,47]; 
    mapTopology.network(48+i*numSpacialStates,:) = [0,49,46,2,48]; 
    mapTopology.network(49+i*numSpacialStates,:) = [48,11,0,0,49]; 
end

% Write to File the top comments and warnings
fprintf(fid,'# This POMDP Model is generated Using Tarek''s MATLAB Script');
fprintf(fid,'\n# This script is still experimental and bugs might appear');
fprintf(fid,'\n# Tarek Taha - Center of Autonomous Systems, UTS\n');
% Specify Essential Parameters
fprintf(fid,'\ndiscount: %f',discount);
fprintf(fid,'\nvalues: reward');

fprintf(fid,'\n\nstates: ');
for i=1:length(destinations)
    for j=1:numSpacialStates
        fprintf(fid,'s%dd%d ',j,i);
    end
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
    % Find the number of reachable nodes (connected)
    % n = length(find(mapTopology.network(j,:)));  
    n = length(destinations);
    % Uncertainty in the final state given this action is divided
    % equally to the connected neighbouring nodes
    actionNoise = actionsUncertainty(i)/(n-1)/100;    
    if length(find(actionsUncertainty)) ~=0
        actionProb = (100 - actionsUncertainty(i))/100;
        actionNoise = actionsUncertainty(i)/(n-1)/100;
    else
        actionProb  = 1;
        actionNoise = 0;
    end
    for j=1:mapTopology.nnodes
        if mod(j,numSpacialStates) == 0
            state = numSpacialStates;
        else
            state = mod(j,numSpacialStates);
        end
        
        if floor(j/numSpacialStates) == 0 
            dest = 1;
        elseif mod(j,numSpacialStates) == 0
            dest  = floor(j/numSpacialStates);
        else
            dest  = floor(j/numSpacialStates) + 1;
        end
%         % Deterministic Case
        if length(find(actionsUncertainty)) == 0
            if (mapTopology.network(j,i)==0)
                fprintf(fid,'\nT: %s : s%dd%d : s%dd%d %f',actions{i},state,dest,state,dest,actionProb);
            else
                fprintf(fid,'\nT: %s : s%dd%d : s%dd%d %f',actions{i},state,dest,mapTopology.network(j,i),dest,actionProb);
            end
            continue;
        end
        % Allow change of intention by switching in between destinations
        for k=1:n
            % Undefined action to this state
            if (mapTopology.network(j,i)==0)
                fprintf(fid,'\nT: %s : s%dd%d : s%dd%d %f',actions{i},state,dest,state,k,1/n);
                continue;
            end
            if k==dest
                fprintf(fid,'\nT: %s : s%dd%d : s%dd%d %f',actions{i},state,dest,mapTopology.network(j,i),k,actionProb);
            else
                fprintf(fid,'\nT: %s : s%dd%d : s%dd%d %f',actions{i},state,dest,mapTopology.network(j,i),k,actionNoise);
            end
        end
        fprintf(fid,'\n');
    end
    fprintf(fid,'\n#############################################');
end

% Build Observation Probabilities
fprintf(fid,'\n\n# O : <action> : <end-state> : <observation> %%f');
fprintf(fid,'\nO: * : * : * 0.0');
for i=1:length(destinations)
    for j=1:numSpacialStates
        index = (i-1)*numSpacialStates + j;   
        % Find the number of reachable nodes (connected)
        n = length(find(mapTopology.network(index,:)));   
        if length(find(observationsUncertainty)) ~=0
            observationProb = 1/n;
        else
            observationProb = 1;
        end
        %Check the connections
        for s=1:length(actions)
            if (mapTopology.network(index,s)~=0)
                fprintf(fid,'\nO: * : s%dd%d : %s %f',j,i,observations{s},observationProb);
                if length(find(observationsUncertainty)) == 0
                    break;
                end
            end
        end
        fprintf(fid,'\n');    
    end
end

% Build Reward Function
fprintf(fid,'\n\n# R: <action> : <start-state> : <end-state> : <observation> %%f');
fprintf(fid,'\nR: * : * : * : * 0.0');
for i=1:length(destinations)
    for j=1:numSpacialStates
        index = (i-1)*numSpacialStates + j;
        for s=1:length(actions)
            if (mapTopology.network(index,s)==0) % Undefine actions are penalized
                fprintf(fid,'\nR: %s : * : s%dd%d : * -10',actions{s},j,i);
            end
        end
    fprintf(fid,'\n');
    end
end
for i=1:length(destinations)
    fprintf(fid,'\nR: Stop : %s : %s : * 10',destinations{i},destinations{i});
    fprintf(fid,'\nR: *    : * : %s : * 100',destinations{i});
end

fclose(fid);
end
