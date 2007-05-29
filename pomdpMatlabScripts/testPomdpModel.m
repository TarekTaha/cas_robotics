function obsProbs = testPomdpModel(pomdpFileModelName,solutionFile,tasksFile)
% This function tests a PomdpModel given a set of tasks.

file = textread(tasksFile,'%s','delimiter','\n','whitespace',' \b\t','bufsize',100000);
k=0;
% i know it's stupid this way but i will find a proper way later 
obsStrings = {'Up','Down','Right','Left','Nothing'};

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
numendLocations = 0;
for i=1:length(fileTemp)
    numObs=0;
    pat='(\d)+\s*(\S+)\s*';
    [s,f,t]=regexp(fileTemp{i},pat); 
    last = length(t);
    % Find the destination index from the set of given destinations
    dest = str2num(fileTemp{i}(t{last}(1,1):t{last}(1,2)));
    % build the observation set from this task
    for j=1:length(t)
        numObs = numObs + 1;
        obs{i}.obs{numObs}   = fileTemp{i}(t{j}(2,1):t{j}(2,2));
        obs{i}.dest{numObs}  = dest;
        obs{i}.pos{numObs}   = str2num(fileTemp{i}(t{j}(1,1):t{j}(1,2)));         
    end
end
pomdp = readPOMDP(pomdpFileModelName,0);
n = pomdp.nrStates;
currentBelief=zeros(1,n);
for i=1:length(obs)
    % first state is assumed to be known
    currentBelief(1,obs{i}.pos{1}) = 1;    
    for j=1:size(obs{i}.obs,2)      
        currentObs = strfind(obsStrings,obs{i}.obs{j});
        for k=1:length(currentObs)
            if ~isempty(currentObs{k})
                obsIndx = k;
            end 
        end          
        % We don't really care about the previous action at this stage
        prevAction = 1; 
        updatedBelief = updateBelief(pomdpFileModelName,currentBelief,obsIndx,prevAction);
        [value , mostProbableCurrentLocation] = max(updatedBelief);
        [maxValue action] = zmdpParser(solutionFile,updatedBelief);
        [value, index] = max(pomdp.transition(:,mostProbableCurrentLocation,action));
        currentBelief=zeros(1,n);
        currentBelief(index) = 1;
    end
    if currentBelief(index) == obs{i}.dest{1}
        display('Success');
    else
        display('Failure');
    end
end

end