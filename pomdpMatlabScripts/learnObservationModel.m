function obsProbs = learnObservationModel(pomdpModel)
% This function build the observation probabilities from a set of recorded
% data. The function reads a file that contains a task on each line
% representing a sequence of positions and observations recoreded to reach
% a destination. 

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
  % build the observation set from this task
  for j=1:length(t)
      numObs = numObs + 1;
      obs{numObs}.obs   = fileTemp{i}(t{j}(2,1):t{j}(2,2));
      obs{numObs}.dest  = destIndx;
      obs{numObs}.pos   = str2num(fileTemp{i}(t{j}(1,1):t{j}(1,2)));
      obs{numObs}.state = sprintf('s%dd%d',obs{numObs}.pos,destIndx);            
  end
end

% build the probabilities
obsSum = zeros(pomdpModel.numSpatialStates*length(pomdpModel.destinations),length(pomdpModel.observations));
for i=1:length(obs)
    index = (obs{i}.dest-1)*pomdpModel.numSpatialStates + obs{i}.pos;
    indx = strfind(pomdpModel.observations,obs{i}.obs);
    for j=1:length(indx)
        if ~isempty(indx{j})
            obsIndx = j;
        end 
    end    
    obsSum(index,obsIndx) = obsSum(index,obsIndx) + 1;
end 
obsProbs = zeros(pomdpModel.numSpatialStates*length(pomdpModel.destinations),length(pomdpModel.observations));

% Normalize probabilities
for i=1:length(obsProbs)
    for j=1:length(pomdpModel.observations)
        obsProbs(i,j) = obsSum(i,j)/sum(obsSum(i,:));
    end
end
clear obsSum;
clear fileTemp;
end