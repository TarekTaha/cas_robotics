function obsProbs = learnObservationModel(pomdpModel)
% This function build the observation probabilities from a set of recorded
% data. The function reads a file that contains tasks on each line. Each
% task is a sequence of positions and observations recoreded to reach
% a destination. 
%
% Inputs:
% pomdpModel = a structure containing information about the pomdel Model,
% like the actions, observation, number of states ...
% Output:
% obsProbs = a (n,m) Matrix of observation probabilities where 'n' is the
% number of states in the model and 'm' is the number of observations.

% read the file and save in buffer
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
numObs=0;
numendLocations = 0;
for i=1:length(fileTemp)  
  pat='(\d)+\s*(\S+)\s*';
  [s,f,t]=regexp(fileTemp{i},pat); 
  last = length(t);
  % Find the destination index from the set of given destinations
  dest = str2num(fileTemp{i}(t{last}(1,1):t{last}(1,2)));
  indx = strfind(pomdpModel.destinations,sprintf('s%d',dest));
  for j=1:length(indx)
      if ~isempty(indx{j})
        destIndx = j;
      end 
  end
  if ~exist('destIndx')
      error(sprintf('Undefined Destination !!!'));
  end
  % build the observation set from this task
  for j=1:length(t)
      numObs = numObs + 1;
      obs{numObs}.obs   = fileTemp{i}(t{j}(2,1):t{j}(2,2));
      obs{numObs}.dest  = destIndx;
      obs{numObs}.pos   = str2num(fileTemp{i}(t{j}(1,1):t{j}(1,2)));         
  end
end

% build the frequency based probabilities 
obsSum = zeros(pomdpModel.numSpatialStates,length(pomdpModel.observations));
for i=1:length(obs)
    indx = strfind(pomdpModel.observations,sprintf('%s-%d',obs{i}.obs,obs{i}.pos));
    for j=1:length(indx)
        if ~isempty(indx{j})
            obsIndx = j;
        end 
    end
    if ~exist('obsIndx')
        error('Unknown Observation !!!');
    end
    obsSum(obs{i}.pos,obsIndx) = obsSum(obs{i}.pos,obsIndx) + 1;
    clear obsIndx;
end 

% Normalize probabilities
obsProbs = zeros(pomdpModel.numSpatialStates,length(pomdpModel.observations));
for i=1:size(obsProbs,1)
    for j=1:length(pomdpModel.observations)
        obsProbs(i,j) = obsSum(i,j)/sum(obsSum(i,:));
    end
end

clear obsSum;
clear fileTemp;
end