function pomdpModel = modelDefinitions(outputFile,obsDataFile)

pomdpModel.outputFile  = outputFile;
pomdpModel.obsDataFile = obsDataFile;
pomdpModel.discount = 0.95;
pomdpModel.numSpatialStates = 16;
pomdpModel.obsCrossFeature  = 0;
pomdpModel.destCrossFeature = 1;

% define the destinations
pomdpModel.destinations = {'s1d1','s7d2','s8d3','s16d4'};
pomdpModel.actions = {'North','South','East','West','Stop'};
pomdpModel.obsStrings = {'Up','Down','Right','Left','Nothing'};

% Percentage Uncertainty in the actions
%pomdpModel.actionsUncertainty = [10,10,20,20,10];    
pomdpModel.actionsUncertainty = [0,0,0,0,0];

% Percentage Uncertainty in the Observations
%pomdpModel.observationsUncertainty = [10,10,10,10,15];
pomdpModel.observationsUncertainty = [10,5,15,10,20];

% Construct the Graph network by defining the number of states and the
% translation inbetween the states. This will be used to build the
% translational probobilities with added uncertainties.

pomdpModel.mapTopology.nnodes = pomdpModel.numSpatialStates;
pomdpModel.mapTopology.network = zeros(pomdpModel.mapTopology.nnodes,length(pomdpModel.actions));

%                                      N S E W No    
pomdpModel.mapTopology.network(1,:)  = [0,2,0,0,1]; 
pomdpModel.mapTopology.network(2,:)  = [1,3,8,0,2]; 
pomdpModel.mapTopology.network(3,:)  = [2,4,0,0,3]; 
pomdpModel.mapTopology.network(4,:)  = [3,5,0,0,4]; 
pomdpModel.mapTopology.network(5,:)  = [4,0,6,0,5]; 
pomdpModel.mapTopology.network(6,:)  = [7,0,0,5,6]; 
pomdpModel.mapTopology.network(7,:)  = [0,6,0,0,7]; 
pomdpModel.mapTopology.network(8,:)  = [9,0,0,2,8]; 
pomdpModel.mapTopology.network(9,:)  = [10,8,12,0,9];
pomdpModel.mapTopology.network(10,:) = [0,9,0,0,10];
pomdpModel.mapTopology.network(11,:) = [12,0,0,0,11];
pomdpModel.mapTopology.network(12,:) = [13,11,15,9,12];
pomdpModel.mapTopology.network(13,:) = [0,12,0,0,13];
pomdpModel.mapTopology.network(14,:) = [15,0,0,0,14];
pomdpModel.mapTopology.network(15,:) = [16,14,0,12,15];
pomdpModel.mapTopology.network(16,:) = [0,15,0,0,16];   

end