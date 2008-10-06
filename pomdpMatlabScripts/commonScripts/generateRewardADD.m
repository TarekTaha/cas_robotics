function rewardFunction = generateRewardADD(tasksFile)
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

% Build Reward Function
% read the file and save in buffer

% Get preferred routes from the training data and reward them
file = textread(tasksFile,'%s','delimiter','\n','whitespace',' \b\t','bufsize',100000);
k=0;

destinations = {'s1d1','s6d2','s44d3','s30d4','s26d5','s38d6'};
obsStrings = {'Up','Down','Right','Left','Nothing'};
numSpatialStates = 49;

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
  indx = strfind(destinations,sprintf('s%dd',dest));
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
      reward{num}.start      = sprintf('s%d',str2num(fileTemp{i}(t{j-1}(1,1):t{j-1}(1,2))));
      reward{num}.end        = sprintf('s%d',str2num(fileTemp{i}(t{j}(1,1):t{j}(1,2))));
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
    rewardSum{s}.totalVisited = 0;
    for j=i:length(reward)
        if strcmp(reward{i}.start,reward{j}.start) && strcmp(reward{i}.end,reward{j}.end) && (reward{i}.dest == reward{j}.dest) && ...
           strcmp(reward{i}.obs,reward{j}.obs) %&& (reward{i}.taskStart == reward{j}.taskStart)
            rewardSum{s}.obs          = reward{i}.obs;
            rewardSum{s}.start        = reward{i}.start;
            rewardSum{s}.end          = reward{i}.end;        
            rewardSum{s}.dest         = reward{i}.dest;        
            rewardSum{s}.taskStart    = reward{i}.taskStart;
            rewardSum{s}.totalVisited = rewardSum{s}.totalVisited + 1;
            rewardSum{s}.reward       = rewardSum{s}.reward + 10;
            rewardSum{s}.visited      = 0;            
            reward{j}.visited         = 1;
        end
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
        if strcmp(rewardSum{i}.start,rewardSum{j}.start) && (rewardSum{i}.dest == rewardSum{j}.dest) %&& (rewardSum{i}.taskStart == rewardSum{j}.taskStart)
            total{s}.reward      = total{s}.reward + rewardSum{j}.reward;
            total{s}.start       = rewardSum{j}.start;
            total{s}.dest        = rewardSum{j}.dest;
            total{s}.taskStart   = rewardSum{j}.taskStart;
            rewardSum{j}.visited = 1;
        end
    end
end

% Normalize the total reward at each step to 10 and divide it to the rest
% of alternative routes according to frequncy.
% Print the rewards to the file
fileid = fopen('rewardOutput.txt','w');

normalizeTo = 10;
%fprintf(fid,'\n\n# R: <action> : <start-state> : <end-state> : <observation> %%f');
%fprintf(fid,'\nR: * : * : * : * -100.0');

fprintf(fileid,'dd destPrediction\n');
fprintf(fileid,'\t(loc\n');
for i=1:length(rewardSum)
    for j=1:length(total)
        if strcmp(rewardSum{i}.start,total{j}.start) && (rewardSum{i}.dest == total{j}.dest) %&& (rewardSum{i}.taskStart == total{j}.taskStart)
            result= normalizeTo*rewardSum{i}.reward/total{j}.reward;
            if(result > 10)
                reply = input('Unexpected Error? Y/N [Y]: ', 's');
            end
            fprintf(fileid,'\t\t(%s\t(destination''\t(d%d\t(loc''\t\t(%s\t(intention''\t(%s (%f))))))))\n',rewardSum{i}.start,rewardSum{i}.dest,rewardSum{i}.end,rewardSum{i}.obs,result);
            break;
        end
    end
end

clear fileTemp;
fclose(fileid);