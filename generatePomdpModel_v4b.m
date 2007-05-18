function generatePomdpModel(modelFileName,observationsDataFile)
% This function generated the POMDP model according to Tony Cassandra's
% syntaxt. You have to specify the actions, observations and the
% uncertainty in their readings.

fid = fopen(modelFileName, 'wb');
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
destinations = {'s3d1','s7d2'};
numSpacialStates = 9;
% Here we are building a model that is a cross product between states
% representiong spaces in the environment and destinations that the user
% wants to go to.

mapTopology.nnodes = numSpacialStates*length(destinations);
mapTopology.network = zeros(mapTopology.nnodes,length(actions));
    
for i=0:(length(destinations)-1)
    mapTopology.network(1+i*numSpacialStates,:) = [0,4,0,0,1]; 
    mapTopology.network(2+i*numSpacialStates,:) = [0,5,0,0,2]; 
    mapTopology.network(3+i*numSpacialStates,:) = [0,6,0,0,3]; 
    mapTopology.network(4+i*numSpacialStates,:) = [1,7,5,0,4]; 
    mapTopology.network(5+i*numSpacialStates,:) = [2,8,6,4,5]; 
    mapTopology.network(6+i*numSpacialStates,:) = [3,9,0,5,6]; 
    mapTopology.network(7+i*numSpacialStates,:) = [4,0,0,0,7]; 
    mapTopology.network(8+i*numSpacialStates,:) = [5,0,0,0,8]; 
    mapTopology.network(9+i*numSpacialStates,:) = [6,0,0,0,9];
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
    % actionNoise = actionsUncertainty(i)/(n-1)/100;    
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

% Manually Build The Observation Probabilities

fprintf(fid,'\n\n# O : <action> : <end-state> : <observation> %%f');
fprintf(fid,'\nO: * : * : * 0.0');

file = textread(observationsDataFile,'%s','delimiter','\n','whitespace',' \b\t','bufsize',100000);
k=0;
for i=1:length(file)
  comment=strfind(file{i},'#');
  if ~isempty(comment)
    file{i}(comment(1):end)=[];
  end
  if ~isempty(file{i})
    k=k+1;
    fileTemp{k}=file{i};
  end
end
clear file;

for i=1:length(fileTemp)  
    obs{i} = sscanf(fileTemp{i},'%d %s');
end

fprintf(fid,'\nO: * : s1d1 : Down 0.9');
fprintf(fid,'\nO: * : s1d1 : Nothing 0.1');
fprintf(fid,'\nO: * : s1d2 : Down 0.9');
fprintf(fid,'\nO: * : s1d2 : Nothing 0.1');

fprintf(fid,'\nO: * : s2d1 : Down 0.9');
fprintf(fid,'\nO: * : s2d1 : Nothing 0.1');
fprintf(fid,'\nO: * : s2d2 : Down 0.9');
fprintf(fid,'\nO: * : s2d2 : Nothing 0.1');

fprintf(fid,'\nO: * : s3d1 : Down 0.1');
fprintf(fid,'\nO: * : s3d1 : Nothing 0.9');
fprintf(fid,'\nO: * : s3d2 : Down 0.9');
fprintf(fid,'\nO: * : s3d2 : Nothing 0.1');

fprintf(fid,'\nO: * : s4d1 : Right 0.9');
fprintf(fid,'\nO: * : s4d1 : Nothing 0.1');
fprintf(fid,'\nO: * : s4d2 : Down 0.9');
fprintf(fid,'\nO: * : s4d2 : Nothing 0.1');

fprintf(fid,'\nO: * : s5d1 : Right 0.9');
fprintf(fid,'\nO: * : s5d1 : Nothing 0.1');
fprintf(fid,'\nO: * : s5d2 : Left 0.9');
fprintf(fid,'\nO: * : s5d2 : Nothing 0.1');

fprintf(fid,'\nO: * : s6d1 : Up 0.9');
fprintf(fid,'\nO: * : s6d1 : Nothing 0.1');
fprintf(fid,'\nO: * : s6d2 : Left 0.9');
fprintf(fid,'\nO: * : s6d2 : Nothing 0.1');

fprintf(fid,'\nO: * : s7d1 : Up 0.9');
fprintf(fid,'\nO: * : s7d1 : Nothing 0.1');
fprintf(fid,'\nO: * : s7d2 : Up 0.1');
fprintf(fid,'\nO: * : s7d2 : Nothing 0.9');

fprintf(fid,'\nO: * : s8d1 : Up 0.9');
fprintf(fid,'\nO: * : s8d1 : Nothing 0.1');
fprintf(fid,'\nO: * : s8d2 : Up 0.9');
fprintf(fid,'\nO: * : s8d2 : Nothing 0.1');

fprintf(fid,'\nO: * : s9d1 : Up 0.9');
fprintf(fid,'\nO: * : s9d1 : Nothing 0.1');
fprintf(fid,'\nO: * : s9d2 : Up 0.9');
fprintf(fid,'\nO: * : s9d2 : Nothing 0.1');

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
    end
    fprintf(fid,'\n');
end
for i=1:length(destinations)
    fprintf(fid,'\nR: Stop : %s : %s : * 10',destinations{i},destinations{i});
    fprintf(fid,'\nR: *    : *  : %s : * 100',destinations{i});
end
clear fileTemp;
fclose(fid);
end

function r = expandString(c,members)
  r = strmatch(c,members,'exact');
  if isempty(r) % apparently c is a numbered state
    r = str2double(c)+1; % Matlab starts at 1, not 0
  end
end