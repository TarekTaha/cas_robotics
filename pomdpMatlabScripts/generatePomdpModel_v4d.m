function generatePomdpModel(modelFileName,observationsDataFile)
% This function generated the POMDP model according to Tony Cassandra's
% syntaxt. You have to specify the actions, observations and the
% uncertainty in their readings.

pomdpModel.outputFile  = modelFileName;
pomdpModel.obsDataFile = observationsDataFile;

fid = fopen(modelFileName, 'wb');
global mapTopology;
discount = 0.95;
pomdpModel.numSpatialStates = 9;

% define the destinations
pomdpModel.destinations = {'s3','s7'};

pomdpModel.actions = {'North','South','East','West','Stop'};
observations = {'Up','Down','Right','Left','Nothing'};
numObs = 0;
for i=1:pomdpModel.numSpatialStates
    for j=1:length(observations)
        numObs = numObs + 1;
        %pomdpModel.observations{numObs} = sprintf('%d-%s',i,observations{j});
        pomdpModel.observations{numObs} = sprintf('%s-%d',observations{j},i);
    end
end

% Percentage Uncertainty in the actions
%pomdpModel.actionsUncertainty = [10,10,20,20,10];    
pomdpModel.actionsUncertainty = [0,0,0,0,0];
% Percentage Uncertainty in the Observations
%pomdpModel.observationsUncertainty = [10,10,10,10,15];
pomdpModel.observationsUncertainty = [0,0,0,0,0];

% Construct the Graph network by defining the number of states and the
% translation inbetween the states. This will be used to build the
% translational probobilities with added uncertainties.



% Here we are building a model that is a cross product between states
% representiong spaces in the environment and destinations that the user
% wants to go to.

mapTopology.nnodes = pomdpModel.numSpatialStates;
mapTopology.network = zeros(mapTopology.nnodes,length(pomdpModel.actions));
    

    mapTopology.network(1,:) = [0,4,0,0,1]; 
    mapTopology.network(2,:) = [0,5,0,0,2]; 
    mapTopology.network(3,:) = [0,6,0,0,3]; 
    mapTopology.network(4,:) = [1,7,5,0,4]; 
    mapTopology.network(5,:) = [2,8,6,4,5]; 
    mapTopology.network(6,:) = [3,9,0,5,6]; 
    mapTopology.network(7,:) = [4,0,0,0,7]; 
    mapTopology.network(8,:) = [5,0,0,0,8]; 
    mapTopology.network(9,:) = [6,0,0,0,9];


% Write to File the top comments and warnings
fprintf(fid,'# This POMDP Model is generated Using Tarek''s MATLAB Script');
fprintf(fid,'\n# This script is still experimental and bugs might appear');
fprintf(fid,'\n# Tarek Taha - Center of Autonomous Systems, UTS\n');
% Specify Essential Parameters
fprintf(fid,'\ndiscount: %f',discount);
fprintf(fid,'\nvalues: reward');

fprintf(fid,'\n\nstates: ');
for i=1:pomdpModel.numSpatialStates
    fprintf(fid,' s%d',i);
end


fprintf(fid,'\n\nactions:');
for i=1:length(pomdpModel.actions)
    fprintf(fid,' %s',pomdpModel.actions{i});
end
fprintf(fid,'\n\nobservations:');
for i=1:length(pomdpModel.observations)
    fprintf(fid,' %s',pomdpModel.observations{i});
end
startBelief = 1.0/mapTopology.nnodes;
fprintf(fid,'\nstart:');
for i=1:mapTopology.nnodes
    fprintf(fid,' %f',startBelief);
end

% Build the Translation probabilities
fprintf(fid,'\n\n# T: <action> : <start-state> : <end-state> %%f');
fprintf(fid,'\nT: * : * : * 0.0');
for i=1:length(pomdpModel.actions)
    % Find the number of reachable nodes (connected)
    % n = length(find(mapTopology.network(j,:)));  
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
    for j=1:mapTopology.nnodes
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
%         % Deterministic Case
        if length(find(pomdpModel.actionsUncertainty)) == 0
            if (mapTopology.network(j,i)==0)
                fprintf(fid,'\nT: %s : s%d : s%d %f',pomdpModel.actions{i},state,state,actionProb);
            else
                fprintf(fid,'\nT: %s : s%d : s%d %f',pomdpModel.actions{i},state,mapTopology.network(j,i),actionProb);
            end
            continue;
        end
        % Allow change of intention by switching in between destinations
        for k=1:n
            % Undefined action to this state
            if (mapTopology.network(j,i)==0)
                fprintf(fid,'\nT: %s : s%d : s%d %f',pomdpModel.actions{i},state,state,1/n);
                continue;
            end
            if k==dest
                fprintf(fid,'\nT: %s : s%d : s%d %f',pomdpModel.actions{i},state,mapTopology.network(j,i),actionProb);
            else
                fprintf(fid,'\nT: %s : s%d : s%d %f',pomdpModel.actions{i},state,mapTopology.network(j,i),actionNoise);
            end
        end
        fprintf(fid,'\n');
    end
    fprintf(fid,'\n#############################################');
end

% Build The Observation Probabilities from a data file

fprintf(fid,'\n\n# O : <action> : <end-state> : <observation> %%f');
fprintf(fid,'\nO: * : * : * 0.0');

[obsProbs] = learnObservationModel2(pomdpModel);

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
        obsProb = obsProbs(i,j)/sum(obsProbs(i,:));
        fprintf(fid,'\nO: * : s%d : %s %f',state,pomdpModel.observations{j},obsProb);
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