function obsProbs = generateObsADD(tasksFile)
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

% read the file and save in buffer

file = textread(tasksFile,'%s','delimiter','\n','whitespace',' \b\t','bufsize',100000);
k=0;

destinations = {'s1d1','s6d2','s44d3','s30d4','s26d5','s38d6'};
obsStrings = {'Up','Down','Right','Left','Nothing'};
numSpatialStates = 49;
observationsUncertainty = [5,5,5,5,5];

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
      numObs = numObs + 1;
      obs{numObs}.obs   = fileTemp{i}(t{j}(2,1):t{j}(2,2));
      obs{numObs}.dest  = destIndx;
      obs{numObs}.pos   = str2num(fileTemp{i}(t{j}(1,1):t{j}(1,2)));         
      %obs{numObs}.prevAction = 
  end
end

% build the frequency based probabilities 
obsSum = zeros(numSpatialStates*length(destinations),length(obsStrings));
for i=1:length(obs)
    index = (obs{i}.dest-1)*numSpatialStates + obs{i}.pos;    
    
    indx = strfind(obsStrings,obs{i}.obs);

    for j=1:length(indx)
        if ~isempty(indx{j})
            obsIndx = j;
        end 
    end   
    if ~exist('obsIndx')
        error('Undefined Observation');
    end
    obsSum(index,obsIndx) = obsSum(index,obsIndx) + 1;
end 

% Normalize probabilities and add uncertainty
uncertainty = 0;
obsProbs = zeros(numSpatialStates*length(destinations),length(obsStrings));
for i=1:size(obsProbs,1)
    uncertainty = 0;
    n = length(obsStrings) - length(find(obsSum(i,:)));
    % Build Uncertainty and construct Observations with noise
    for j=1:length(obsStrings)
        if sum(obsSum(i,:)) ~= 0
            obsProbs(i,j) = obsSum(i,j)/sum(obsSum(i,:));
        else
            obsProbs(i,j) = 0;
        end

        temp = obsStrings{j};
        indx = strfind(obsStrings,temp);
        for k=1:length(indx)
            if ~isempty(indx{k})
                obsIndx = k;
            end 
        end
        
        uncertainty = uncertainty + obsProbs(i,j)*observationsUncertainty(obsIndx)/100;
        if obsProbs(i,j) ~=0
            obsProbs(i,j) = obsProbs(i,j) - obsProbs(i,j)*observationsUncertainty(obsIndx)/100;
        end
    end
    % Uniformly distribute Uncertainty over other Observations
    for j=1:length(obsStrings)
        if (obsProbs(i,j) == 0) && (sum(obsSum(i,:)) ~= 0)
            obsProbs(i,j) = uncertainty/n;
        elseif (obsProbs(i,j) == 0) && (sum(obsSum(i,:)) == 0)
            obsProbs(i,j) = 1/length(obsStrings); 
        end
    end
end

% build the frequency based probabilities of destinations
destSum = zeros(numSpatialStates*length(obsStrings),length(destinations));
for i=1:length(obs)
    indx = strfind(obsStrings,obs{i}.obs);
    for j=1:length(indx)
        if ~isempty(indx{j})
            obsIndx = j;
        end 
    end      
    index = (obsIndx-1)*numSpatialStates + obs{i}.pos;    
    destSum(index,obs{i}.dest) = destSum(index,obs{i}.dest) + 1;
end 

destProbs= zeros(numSpatialStates*length(obsStrings),length(destinations));
for i=1:size(destProbs,1)
    n = length(destinations) - length(find(destSum(i,:)));
    % Build Uncertainty and construct Observations with noise
    for j=1:length(destinations)
        if sum(destSum(i,:)) ~= 0
            destProbs(i,j) = destSum(i,j)/sum(destSum(i,:));
        else
            destProbs(i,j) = 0;
        end
    end
end

fileid = fopen('obsOutput.txt','w');
fprintf(fileid,'dd destPrediction\n');
fprintf(fileid,'\t(loc''\n');

for i=1:numSpatialStates
   fprintf(fileid,'\t\t(s%d\n\t\t\t(intention''\n',i);
   for j=1:length(obsStrings)
        indx = (i+(j-1)*numSpatialStates);
        if sum(destProbs(indx,:))==0
            destProbs(indx,:) = 1/6;
        end
        fprintf(fileid,'\t\t\t\t(%-10s(destination'' (d1 (%f)) (d2(%f)) (d3 (%f)) (d4 (%f)) (d5 (%f)) (d6 (%f)) ))\n',obsStrings{j},destProbs(indx,1),destProbs(indx,2),destProbs(indx,3),destProbs(indx,4),destProbs(indx,5),destProbs(indx,6));			
   end
   fprintf(fileid,'\t\t\t)\n\t\t)\n');   
end

% for i=1:numSpatialStates
%    for j=1:length(destinations)
%         indx = (i+(j-1)*numSpatialStates);
%         if(j==1)
%             fprintf(fileid,'\t\t(s%-4d\t(destination''\t(d%-4d\t(intention'' (Up (%f)) (Down (%f)) (Right (%f)) (Left (%f)) (Nothing (%f)) ))\n',i,j,obsProbs(indx,1),obsProbs(indx,2),obsProbs(indx,3),obsProbs(indx,4),obsProbs(indx,5));			
%         else
%             fprintf(fileid,'\t\t\t\t\t(d%-4d\t(intention'' (Up (%f)) (Down (%f)) (Right (%f)) (Left (%f)) (Nothing (%f)) ))\n',j,obsProbs(indx,1),obsProbs(indx,2),obsProbs(indx,3),obsProbs(indx,4),obsProbs(indx,5));			        
%         end
%    end
%    fprintf(fileid,'\t\t\t)\n\t\t)\n');   
% end

fprintf(fileid,'\t)\nenddd');
fclose(fileid);

clear obsSum;
clear fileTemp;
end