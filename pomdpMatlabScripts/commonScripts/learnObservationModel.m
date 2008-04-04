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
      numObs = numObs + 1;
      obs{numObs}.obs   = fileTemp{i}(t{j}(2,1):t{j}(2,2));
      obs{numObs}.dest  = destIndx;
      obs{numObs}.pos   = str2num(fileTemp{i}(t{j}(1,1):t{j}(1,2)));         
      %obs{numObs}.prevAction = 
  end
end

% build the frequency based probabilities 
obsSum = zeros(pomdpModel.numSpatialStates*length(pomdpModel.destinations),length(pomdpModel.observations));
for i=1:length(obs)
    index = (obs{i}.dest-1)*pomdpModel.numSpatialStates + obs{i}.pos;    
    
    if (pomdpModel.obsCrossFeature)
        indx = strfind(pomdpModel.observations,sprintf('%s-%d',obs{i}.obs,obs{i}.pos));
    else
        indx = strfind(pomdpModel.observations,obs{i}.obs);
    end

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
obsProbs = zeros(pomdpModel.numSpatialStates*length(pomdpModel.destinations),length(pomdpModel.observations));
for i=1:size(obsProbs,1)
    uncertainty = 0;
    n = length(pomdpModel.observations) - length(find(obsSum(i,:)));
    % Build Uncertainty and construct Observations with noise
    for j=1:length(pomdpModel.observations)
        if sum(obsSum(i,:)) ~= 0
            obsProbs(i,j) = obsSum(i,j)/sum(obsSum(i,:));
        else
            obsProbs(i,j) = 0;
        end

        % Link the uncertainty to the observation
        if (pomdpModel.obsCrossFeature)
            temp = pomdpModel.observations{j}(1:strfind(pomdpModel.observations{j},'-')-1);
        else
            temp = pomdpModel.observations{j};
        end      
        indx = strfind(pomdpModel.obsStrings,temp);
        for k=1:length(indx)
            if ~isempty(indx{k})
                obsIndx = k;
            end 
        end
        
        uncertainty = uncertainty + obsProbs(i,j)*pomdpModel.observationsUncertainty(obsIndx)/100;
        if obsProbs(i,j) ~=0
            obsProbs(i,j) = obsProbs(i,j) - obsProbs(i,j)*pomdpModel.observationsUncertainty(obsIndx)/100;
        end
    end
    % Uniformly distribute Uncertainty over other Observations
    for j=1:length(pomdpModel.observations)
        if (obsProbs(i,j) == 0) && (sum(obsSum(i,:)) ~= 0)
            obsProbs(i,j) = uncertainty/n;
        elseif (obsProbs(i,j) == 0) && (sum(obsSum(i,:)) == 0)
            obsProbs(i,j) = 1/length(pomdpModel.observations); 
        end
    end
end

clear obsSum;
clear fileTemp;
end