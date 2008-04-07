function testTasksOnline(pomdpFileModelName,tasksFile)
% This function tests a PomdpModel given a set of tasks.

% ************************************************************************\
% * Copyright (C) 2007 - 2008 by:                                         *
% *    Tarek Taha, Jaime Valls Miro, Gamini Dissanayake                   *
% *     CAS-UTS  {t.taha,j.vallsmiro,g.dissanayake}@cas.edu.au            *
% *                                                                       *
% *                                                                       *
% * This program is free software; you can redistribute it and/or modify  *
% * it under the terms of the GNU General Public License as published by  *
% * the Free Software Foundation; either version 2 of the License, or     *
% * (at your option) any later version.                                   *
% *                                                                       *
% * This program is distributed in the hope that it will be useful,       *
% * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
% * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
% * GNU General Public License for more details.                          *
% *                                                                       *
% * You should have received a copy of the GNU General Public License     *
% * along with this program; if not, write to the                         *
% * Free Software Foundation, Inc.,                                       *
% * 51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
% ************************************************************************/

file = textread(tasksFile,'%s','delimiter','\n','whitespace',' \b\t','bufsize',100000);
k=0;
% i know it's stupid this way but i will find a proper way later 
obsStrings  = {'Up','Down','Right','Left','Nothing'};
destStrings = {'s1d1','s6d2','s26d3','s30d4','s31d5','s38d6'};

global pomdp;
pomdp  = readPOMDP(pomdpFileModelName,0);

simplifyReward();

display('POMDP Model File read.')

n = pomdp.nrStates;
m = pomdp.nrStates/length(destStrings);

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
    if isempty(destIndx)
        error('Unknown Destination !!!');
    end
    % build the observation set from this task
    for j=1:length(t)
        numObs = numObs + 1;
        obs{i}.obs{numObs}   = fileTemp{i}(t{j}(2,1):t{j}(2,2));
        obs{i}.dest{numObs}  = destIndx;
        obs{i}.pos{numObs}   = str2num(fileTemp{i}(t{j}(1,1):t{j}(1,2))) + (destIndx-1)*m;         
    end
end
failure = 0;
success = 0;
D = 2;
for i=1:5%length(obs)
    % first state is assumed to be known
    currentBelief=zeros(1,n);
    currentBelief(1,obs{i}.pos{1}) = 1;
    factoredBelief = factorStateSpace(pomdp,currentBelief);
    prevAction = 5; % Nothing Action
%     if i==4
%         keyboard
%     end
    display(sprintf('Task:%d',i));
    for j=1:size(obs{i}.obs,2)   
        % Take an Observation
        currentObs = strfind(obsStrings,obs{i}.obs{j});
        for k=1:length(currentObs)
            if ~isempty(currentObs{k})
                obsIndx = k;
            end 
        end      
        % Update the Belief
        updatedBelief = updateFactoredBelief(pomdp,factoredBelief,obsIndx,prevAction);
        [value , mostProbableCurrentLocation] = max(updatedBelief(:).value);
        % Determine the action
        [value action] = RTBSS(pomdp,factoredBelief,2,D);
        %action
        prevAction = action; 
        [value , mostProbableCurrentLocation] = max(updatedBelief(:).value);
        % See where this action will take us
        [value, index] = max(pomdp.transition(:,mostProbableCurrentLocation,action));
        %x = mod(index,49)
        currentBelief=zeros(1,n);
        currentBelief(index) = value;
        factordBelief = factorStateSpace(pomdp,currentBelief);
    end
    destIndx = floor(index/m) + 1;
    if destIndx == obs{i}.dest{1}
        display(sprintf('\tSuccess in Task:%d Destination:%d',i,destIndx));
        success = success + 1;
    else
        display(sprintf('\tFailure in Task:%d Dest reached:%d Intended:%d ',i,destIndx,obs{i}.dest{1}));
        failure = failure +1 ;
    end
    display(sprintf('Failure percentage = %f',failure/success));
end

end