function pomdpModel = modelDefinitions(outputFile,obsDataFile)

pomdpModel.outputFile  = outputFile;
pomdpModel.obsDataFile = obsDataFile;

global mapTopology;
pomdpModel.discount = 0.95;
pomdpModel.numSpatialStates = 9;

% define the destinations
pomdpModel.destinations = {'s3d1','s7d2'};

pomdpModel.actions = {'North','South','East','West','Stop'};
pomdpModel.observations = {'Up','Down','Right','Left','Nothing'};
pomdpModel.obsStrings = pomdpModel.observations;
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

pomdpModel.mapTopology.nnodes = pomdpModel.numSpatialStates*length(pomdpModel.destinations);
pomdpModel.mapTopology.network = zeros(pomdpModel.mapTopology.nnodes,length(pomdpModel.actions));
    
for i=0:(length(pomdpModel.destinations)-1)
    pomdpModel.mapTopology.network(1+i*pomdpModel.numSpatialStates,:) = [0,4,0,0,1]; 
    pomdpModel.mapTopology.network(2+i*pomdpModel.numSpatialStates,:) = [0,5,0,0,2]; 
    pomdpModel.mapTopology.network(3+i*pomdpModel.numSpatialStates,:) = [0,6,0,0,3]; 
    pomdpModel.mapTopology.network(4+i*pomdpModel.numSpatialStates,:) = [1,7,5,0,4]; 
    pomdpModel.mapTopology.network(5+i*pomdpModel.numSpatialStates,:) = [2,8,6,4,5]; 
    pomdpModel.mapTopology.network(6+i*pomdpModel.numSpatialStates,:) = [3,9,0,5,6]; 
    pomdpModel.mapTopology.network(7+i*pomdpModel.numSpatialStates,:) = [4,0,0,0,7]; 
    pomdpModel.mapTopology.network(8+i*pomdpModel.numSpatialStates,:) = [5,0,0,0,8]; 
    pomdpModel.mapTopology.network(9+i*pomdpModel.numSpatialStates,:) = [6,0,0,0,9];
end

end