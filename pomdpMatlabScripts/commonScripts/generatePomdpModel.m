function generatePomdpModel(pomdpModel)
% This function generated the POMDP model according to Tony Cassandra's
% syntax. You have to specify the actions, observations and the
% uncertainty in their readings.

format long;

% if ~exist('pomdpModel.obsCrossFeature')
%     pomdpModel.obsCrossFeature = 0;
% end
% 
% if ~exist('pomdpModel.destCrossFeature')
%     pomdpModel.destCrossFeature = 0;
% end

% If the observations are unique to each spacial State, it's a way to
% encapsulate location to the observation.
if pomdpModel.obsCrossFeature
    numObs = 0;
    for i=1:pomdpModel.numSpatialStates
        for j=1:length(pomdpModel.obsStrings)
            numObs = numObs + 1;
            pomdpModel.observations{numObs} = sprintf('%s-%d',pomdpModel.obsStrings{j},i);
        end
    end
else
    pomdpModel.observations = pomdpModel.obsStrings;
end

% Here we are building a model that is a cross product between states
% representiong spaces in the environment and destinations that the user
% wants to go to.
if pomdpModel.destCrossFeature
    pomdpModel.mapTopology.nnodes = pomdpModel.numSpatialStates*length(pomdpModel.destinations);
    %pomdpModel.mapTopology.network = zeros(pomdpModel.mapTopology.nnodes,length(pomdpModel.actions)); 
    tempMatrix = zeros(pomdpModel.mapTopology.nnodes,length(pomdpModel.actions));
    
    for i=0:(length(pomdpModel.destinations)-1)
        for j=1:pomdpModel.numSpatialStates
            tempMatrix(j+i*pomdpModel.numSpatialStates,:) = pomdpModel.mapTopology.network(j,:); 
        end
    end
    
    pomdpModel.mapTopology.network = tempMatrix;
    clear tempMatrix;
end

fid = fopen(pomdpModel.outputFile, 'wb');

% Write to File the top comments and warnings
fprintf(fid,'# This POMDP Model is generated Using Tarek''s MATLAB Script');
fprintf(fid,'\n# This script is still experimental and bugs might appear');
fprintf(fid,'\n# Tarek Taha - Center of Autonomous Systems, UTS\n');

% Specify Essential Parameters
fprintf(fid,'\ndiscount: %f',pomdpModel.discount);
fprintf(fid,'\nvalues: reward');

fprintf(fid,'\n\nstates: ');
for i=1:length(pomdpModel.destinations)
    for j=1:pomdpModel.numSpatialStates
        fprintf(fid,'s%dd%d ',j,i);
    end
end

fprintf(fid,'\n\nactions:');
for i=1:length(pomdpModel.actions)
    fprintf(fid,' %s',pomdpModel.actions{i});
end
fprintf(fid,'\n\nobservations:');
for i=1:length(pomdpModel.observations)
    fprintf(fid,' %s',pomdpModel.observations{i});
end
startBelief = 1.0/pomdpModel.mapTopology.nnodes;
fprintf(fid,'\nstart:');
for i=1:pomdpModel.mapTopology.nnodes
    fprintf(fid,' %.12f',startBelief);
end

% Build the Translation probabilities
fprintf(fid,'\n\n# T: <action> : <start-state> : <end-state> %%f');
fprintf(fid,'\nT: * : * : * 0.0');
for i=1:length(pomdpModel.actions)
    % Find the number of reachable nodes (connected)
    % n = length(find(pomdpModel.mapTopology.network(j,:)));  
    n = length(pomdpModel.destinations);
    
    % Uncertainty in the final state given this action is divided
    % equally to the connected neighbouring nodes
    % actionNoise = pomdpModel.actionsUncertainty(i)/(n-1)/100;    
    if length(find(pomdpModel.actionsUncertainty)) ~=0
        actionProb = (100 - pomdpModel.actionsUncertainty(i))/100;
        actionNoise = pomdpModel.actionsUncertainty(i)/(n-1)/100;
    else
        actionProb  = 1;
        actionNoise = 0;
    end
    for j=1:pomdpModel.mapTopology.nnodes
        if mod(j,pomdpModel.numSpatialStates) == 0
            state = pomdpModel.numSpatialStates;
        else
            state = mod(j,pomdpModel.numSpatialStates);
        end
        
        if floor(j/pomdpModel.numSpatialStates) == 0 
            dest = 1;
        elseif mod(j,pomdpModel.numSpatialStates) == 0
            dest  = floor(j/pomdpModel.numSpatialStates);
        else
            dest  = floor(j/pomdpModel.numSpatialStates) + 1;
        end
        % Deterministic Case
        if length(find(pomdpModel.actionsUncertainty)) == 0
            if (pomdpModel.mapTopology.network(j,i)==0)
                fprintf(fid,'\nT: %s : s%dd%d : s%dd%d %.8f',pomdpModel.actions{i},state,dest,state,dest,actionProb);
            else
                fprintf(fid,'\nT: %s : s%dd%d : s%dd%d %.8f',pomdpModel.actions{i},state,dest,pomdpModel.mapTopology.network(j,i),dest,actionProb);
            end
            continue;
        end
        % Allow change of intention by switching in between destinations
        for k=1:n
            % Undefined action to this state
            if (pomdpModel.mapTopology.network(j,i)==0)
                fprintf(fid,'\nT: %s : s%dd%d : s%dd%d %.8f',pomdpModel.actions{i},state,dest,state,k,1/n);
                continue;
            end
            if k==dest
                fprintf(fid,'\nT: %s : s%dd%d : s%dd%d %.8f',pomdpModel.actions{i},state,dest,pomdpModel.mapTopology.network(j,i),k,actionProb);
            else
                fprintf(fid,'\nT: %s : s%dd%d : s%dd%d %.8f',pomdpModel.actions{i},state,dest,pomdpModel.mapTopology.network(j,i),k,actionNoise);
            end
        end
        fprintf(fid,'\n');
    end
    fprintf(fid,'\n#############################################');
end

% Build The Observation Probabilities from a data file

fprintf(fid,'\n\n# O : <action> : <end-state> : <observation> %%f');
fprintf(fid,'\nO: * : * : * 0.0');

[obsProbs] = learnObservationModel(pomdpModel);

for i=1:size(obsProbs,1)
    if mod(i,pomdpModel.numSpatialStates) == 0
       state = pomdpModel.numSpatialStates;
    else
       state = mod(i,pomdpModel.numSpatialStates);
    end
    if floor(i/pomdpModel.numSpatialStates) == 0 
       dest = 1;
    elseif mod(i,pomdpModel.numSpatialStates) == 0
       dest  = floor(i/pomdpModel.numSpatialStates);
    else
       dest  = floor(i/pomdpModel.numSpatialStates) + 1;
    end
    for j=1:length(pomdpModel.observations)
        obsProb = obsProbs(i,j);
        fprintf(fid,'\nO: * : s%dd%d : %s %.8f',state,dest,pomdpModel.observations{j},obsProb);
    end
    fprintf(fid,'\n');
end

% Build Reward Function
fprintf(fid,'\n\n# R: <action> : <start-state> : <end-state> : <observation> %%f');
fprintf(fid,'\nR: * : * : * : * -1.0');
for i=1:length(pomdpModel.destinations)
    fprintf(fid,'\nR: *    : *  : %s : * 100',pomdpModel.destinations{i});
end

clear fileTemp;
fclose(fid);
end
