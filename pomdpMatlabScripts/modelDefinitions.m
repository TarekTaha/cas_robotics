function pomdpModel = modelDefinitions()

pomdpModel.outputFile  = modelFileName;
pomdpModel.obsDataFile = observationsDataFile;

fid = fopen(modelFileName, 'wb');
global mapTopology;
discount = 0.95;
pomdpModel.numSpatialStates = 9;

% define the destinations
pomdpModel.destinations = {'s3d1','s7d2'};

pomdpModel.actions = {'North','South','East','West','Stop'};
pomdpModel.obsStrings = {'Up','Down','Right','Left','Nothing'};
numObs = 0;
for i=1:pomdpModel.numSpatialStates
    for j=1:length(pomdpModel.obsStrings)
        numObs = numObs + 1;
        pomdpModel.observations{numObs} = sprintf('%s-%d',pomdpModel.obsStrings{j},i);
    end
end

% Percentage Uncertainty in the actions
%pomdpModel.actionsUncertainty = [10,10,20,20,10];    
pomdpModel.actionsUncertainty = [0,0,0,0,0];
% Percentage Uncertainty in the Observations
%pomdpModel.observationsUncertainty = [10,10,10,10,15];
pomdpModel.observationsUncertainty = [10,5,15,10,20];

% Construct the Graph network by defining the number of states and the
% translation inbetween the states. This will be used to build the
% translational probobilities with added uncertainties.



% Here we are building a model that is a cross product between states
% representiong spaces in the environment and destinations that the user
% wants to go to.

mapTopology.nnodes = pomdpModel.numSpatialStates*length(pomdpModel.destinations);
mapTopology.network = zeros(mapTopology.nnodes,length(pomdpModel.actions));
    
for i=0:(length(pomdpModel.destinations)-1)
    mapTopology.network(1+i*pomdpModel.numSpatialStates,:) = [0,4,0,0,1]; 
    mapTopology.network(2+i*pomdpModel.numSpatialStates,:) = [0,5,0,0,2]; 
    mapTopology.network(3+i*pomdpModel.numSpatialStates,:) = [0,6,0,0,3]; 
    mapTopology.network(4+i*pomdpModel.numSpatialStates,:) = [1,7,5,0,4]; 
    mapTopology.network(5+i*pomdpModel.numSpatialStates,:) = [2,8,6,4,5]; 
    mapTopology.network(6+i*pomdpModel.numSpatialStates,:) = [3,9,0,5,6]; 
    mapTopology.network(7+i*pomdpModel.numSpatialStates,:) = [4,0,0,0,7]; 
    mapTopology.network(8+i*pomdpModel.numSpatialStates,:) = [5,0,0,0,8]; 
    mapTopology.network(9+i*pomdpModel.numSpatialStates,:) = [6,0,0,0,9];
end

end