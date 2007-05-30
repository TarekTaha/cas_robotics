function obsProbs = testPomdpModel(pomdpFileModelName,solutionFile,tasksFile)
% This function tests a PomdpModel given a set of tasks.

file = textread(tasksFile,'%s','delimiter','\n','whitespace',' \b\t','bufsize',100000);
k=0;
% i know it's stupid this way but i will find a proper way later 
obsStrings  = {'Up','Down','Right','Left','Nothing'};
destStrings = {'s3d1','s7d2'};

pomdp = readPOMDP(pomdpFileModelName,0);
n = pomdp.nrStates;
m = pomdp.nrStates/2;

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
    indx = strfind(destStrings,sprintf('s%d',dest));
    for j=1:length(indx)
        if ~isempty(indx{j})
            destIndx = j;
        end 
    end
    % build the observation set from this task
    for j=1:length(t)
        numObs = numObs + 1;
        obs{i}.obs{numObs}   = fileTemp{i}(t{j}(2,1):t{j}(2,2));
        obs{i}.dest{numObs}  = destIndx;
        obs{i}.pos{numObs}   = str2num(fileTemp{i}(t{j}(1,1):t{j}(1,2))) + (destIndx-1)*m;         
    end
end

for i=1:length(obs)
    % first state is assumed to be known
    currentBelief=zeros(1,n);
    currentBelief(1,obs{i}.pos{1}) = 1;   
    prevAction = 5; % Nothing Action
%     if i==4
%         keyboard
%     end
    for j=1:size(obs{i}.obs,2)   
        % Take an Observation
        currentObs = strfind(obsStrings,obs{i}.obs{j});
        for k=1:length(currentObs)
            if ~isempty(currentObs{k})
                obsIndx = k;
            end 
        end      
        % Update the Belief
        updatedBelief = updateBelief(pomdp,currentBelief,obsIndx,prevAction);
        [value , mostProbableCurrentLocation] = max(updatedBelief);
        % Determine the action
        [maxValue action] = zmdpParser(solutionFile,updatedBelief);
        prevAction = action; 
        [value , mostProbableCurrentLocation] = max(updatedBelief);
        % See where this action will take us
        [value, index] = max(pomdp.transition(:,mostProbableCurrentLocation,action));
        %index
        currentBelief=zeros(1,n);
        currentBelief(index) = value;
    end
    destIndx = floor(index/m) + 1;
    if destIndx == obs{i}.dest{1}
        display(sprintf('Success in Task:%d Destination:%d',i,destIndx));
    else
        display(sprintf('Failure in Task:%d Dest reached:%d Intended:%d ',i,destIndx,obs{i}.dest{1}));
    end
end

end