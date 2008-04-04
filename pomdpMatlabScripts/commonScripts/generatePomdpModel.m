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
% startBelief = zeros(1,pomdpModel.mapTopology.nnodes);
% state = 1;
% startBelief(1,0*49 + state) = 0.25;
% startBelief(1,1*49 + state) = 0.25;
% startBelief(1,2*49 + state) = 0.25;
% startBelief(1,3*49 + state) = 0.25;
fprintf(fid,'\nstart:');
for i=1:pomdpModel.mapTopology.nnodes
    %fprintf(fid,' %.12f',startBelief(1,i));
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
    if ~isempty(find(pomdpModel.actionsUncertainty, 1))
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
        if isempty(find(pomdpModel.actionsUncertainty, 1))
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
% read the file and save in buffer

% Get preferred routes from the training data and reward them
file = textread(pomdpModel.obsDataFile,'%s','delimiter','\n','whitespace',' \b\t','bufsize',100000);
k=0;

%ignore comments
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

% read tasks (sequence of observations and locations)
num=0;
numendLocations = 0;
for i=1:length(fileTemp)  
  pat='(\d)+\s*(\S+)\s*';
  [s,f,t]=regexp(fileTemp{i},pat); 
  last = length(t);
  % Find the destination index from the set of given destinations
  taskStart= str2num(fileTemp{i}(t{1}(1,1):t{1}(1,2)));
  dest = str2num(fileTemp{i}(t{last}(1,1):t{last}(1,2)));
  indx = strfind(pomdpModel.destinations,sprintf('s%dd',dest));
  for j=1:length(indx)
      if ~isempty(indx{j})
        destIndx = j;
      end 
  end
  if ~exist('destIndx')
      error('Undefined Destination !!!');
  end
  % build the observation set from this task
  
  for j=1:length(t)
      if j==1
          continue;
      end
      num = num + 1;
      reward{num}.action     = fileTemp{i}(t{j-1}(2,1):t{j-1}(2,2));
      reward{num}.taskStart  = taskStart;
      reward{num}.obs        = fileTemp{i}(t{j}(2,1):t{j}(2,2));
      reward{num}.start      = sprintf('s%dd%d',str2num(fileTemp{i}(t{j-1}(1,1):t{j-1}(1,2))),destIndx);
      reward{num}.end        = sprintf('s%dd%d',str2num(fileTemp{i}(t{j}(1,1):t{j}(1,2))),destIndx);
      reward{num}.dest       = destIndx;
      reward{num}.reward     = 0;
      reward{num}.visited    = 0;
      reward{num}.totalVisited = 0;
  end
end

% Sum the individual reward points for each state/destination pair
s = 0;
for i=1:length(reward)
    if reward{i}.visited
        continue;
    end
    s = s+1;
    rewardSum{s}.reward = 0;
    rewardSum{s}.visited = 0;    
    for j=i:length(reward)
        if strcmp(reward{i}.start,reward{j}.start) && strcmp(reward{i}.end,reward{j}.end) && ...
                strcmp(reward{i}.obs,reward{j}.obs) && (reward{i}.taskStart == reward{j}.taskStart)
            rewardSum{s}.obs        = reward{i}.obs;
            rewardSum{s}.start      = reward{i}.start;
            rewardSum{s}.end        = reward{i}.end;        
            rewardSum{s}.reward = rewardSum{s}.reward + 10;
            rewardSum{s}.taskStart = reward{num}.taskStart;
            reward{j}.visited = 1;
        end
    end
    if strcmp(rewardSum{s}.start,'s43d3')&& strcmp(reward{i}.end,'s40d3')
        display(rewardSum{s}.reward);
    end
end

% Calculate the Reward total at each step (used for normalization later on)
s = 0;
for i=1:length(rewardSum)
    if rewardSum{i}.visited
        continue;
    end
    s = s+1;
    total{s}.reward = 0;
    for j=i:length(rewardSum)
        if strcmp(rewardSum{i}.start,rewardSum{j}.start) && (reward{i}.taskStart == reward{j}.taskStart)
            total{s}.reward = total{s}.reward + rewardSum{j}.reward;
            total{s}.start = rewardSum{j}.start;
            rewardSum{j}.visited = 1;
        end
    end
end

% Normalize the total reward at each step to 10 and divide it to the rest
% of alternative routes according to frequncy.
% Print the rewards to the file
normalizeTo = 10;
fprintf(fid,'\n\n# R: <action> : <start-state> : <end-state> : <observation> %%f');
fprintf(fid,'\nR: * : * : * : * -100.0');

for i=1:length(rewardSum)
    for j=1:length(total)
        if strcmp(rewardSum{i}.start,total{j}.start) && (reward{i}.taskStart == reward{j}.taskStart)
            result= normalizeTo*rewardSum{i}.reward/total{j}.reward;
            %fprintf(fid,'\nR: * : %-5s : %-5s : %-8s %-2.3f ',rewardSum{i}.start,rewardSum{i}.end,rewardSum{i}.obs,result);
            fprintf(fid,'\nR: * : %-5s : %-5s : %-5s %-2.3f ',rewardSum{i}.start,rewardSum{i}.end,rewardSum{i}.obs,result);
            break;
        end
    end
end

for i=1:length(pomdpModel.destinations)
    fprintf(fid,'\nR: *    : *  : %s : * 100',pomdpModel.destinations{i});
end

clear fileTemp;
fclose(fid);
end
