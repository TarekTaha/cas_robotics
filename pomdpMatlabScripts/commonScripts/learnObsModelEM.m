function obsProbs = learnObsModelEM()
% This function build the observation probabilities from a set of recorded
% data. The function reads a file that contains tasks on each line. Each
% task is a sequence of positions and observations recoreded to reach
% a destination. 
%
% Inputs:
% tasksFile = a structure containing information about the pomdel Model,
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

outputModelFile = 'paperexperiment.pomdp';
obsDataFile     = '/home/tataha/Matlab Scripts/pomdpMatlabScripts/problemDefinitions/test/exp_tasks_list_large';
pomdpModel = modelDefinitions(outputModelFile,obsDataFile);

% X = zeros(600,2);
% X(1:200,:) = normrnd(0,1,200,2);
% X(201:400,:) = normrnd(0,2,200,2);
% X(401:600,:) = normrnd(0,3,200,2);
% [W,M,V,L] = EM_GM(X,3,[],[],1,[]) 

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

% read the file and save in buffer

file = textread(obsDataFile,'%s','delimiter','\n','whitespace',' \b\t','bufsize',100000);
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
%trainingList = zeros(length(obs),pomdpModel.numSpatialStates*length(pomdpModel.destinations));
trainingList = zeros(length(obs),2);
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
    trainingList(i,1) = index;
    trainingList(i,2) = obsIndx;
    obsSum(index,obsIndx) = obsSum(index,obsIndx) + 1;
end
% Inputs:
%   X(n,d) - input data, n=number of observations, d=dimension of variable
%   k - maximum number of Gaussian components allowed
%   ltol - percentage of the log likelihood difference between 2 iterations ([] for none)
%   maxiter - maximum number of iteration allowed ([] for none)
%   pflag - 1 for plotting GM for 1D or 2D cases only, 0 otherwise ([] for none)
%   Init - structure of initial W, M, V: Init.W, Init.M, Init.V ([] for none)
%
% Ouputs:
%   W(1,k) - estimated weights of GM
%   M(d,k) - estimated mean vectors of GM
%   V(d,d,k) - estimated covariance matrices of GM
%   L - log likelihood of estimates
%
X = trainingList;
k = pomdpModel.numSpatialStates;%*length(pomdpModel.destinations);
d = size(X,2);

Init.W = ones(1,k);
Init.M = zeros(d,k);   Init.M(:,:) = 2.5;
Init.V = zeros(d,d,k); 
Init.L = 0;
[W,M,V,L] = EM_GM(X,k,[],[],1,[]);

obsProbs = L;
clear obsSum;
clear fileTemp;
end