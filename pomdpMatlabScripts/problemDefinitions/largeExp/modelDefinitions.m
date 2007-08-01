function pomdpModel = modelDefinitions(outputFile,obsDataFile)

pomdpModel.outputFile  = outputFile;
pomdpModel.obsDataFile = obsDataFile;
pomdpModel.discount = 0.95;
pomdpModel.numSpatialStates = 49;
pomdpModel.obsCrossFeature  = 0;
pomdpModel.destCrossFeature = 1;

% define the destinations
pomdpModel.destinations = {'s1d1','s6d2','s26d3','s30d4','s31d5','s38d6'};
pomdpModel.actions = {'North','South','East','West','Stop'};
pomdpModel.obsStrings = {'Up','Down','Right','Left','Nothing'};

% Percentage Uncertainty in the actions
%pomdpModel.actionsUncertainty = [10,10,20,20,10];    
pomdpModel.actionsUncertainty = [0,0,0,0,0];

% Percentage Uncertainty in the Observations
%pomdpModel.observationsUncertainty = [0,0,0,0,0];
pomdpModel.observationsUncertainty = [10,5,15,10,20];

% Construct the Graph network by defining the number of states and the
% translation inbetween the states. This will be used to build the
% translational probobilities with added uncertainties.

pomdpModel.mapTopology.nnodes = pomdpModel.numSpatialStates;
pomdpModel.mapTopology.network = zeros(pomdpModel.mapTopology.nnodes,length(pomdpModel.actions));

pomdpModel.mapTopology.network(1,:) = [0,2,0,0,1]; 
pomdpModel.mapTopology.network(2,:) = [1,3,48,0,2]; 
pomdpModel.mapTopology.network(3,:) = [2,4,0,0,3]; 
pomdpModel.mapTopology.network(4,:) = [3,0,7,5,4]; 
pomdpModel.mapTopology.network(5,:) = [0,6,4,0,5]; 
pomdpModel.mapTopology.network(6,:) = [5,0,0,0,6]; 
pomdpModel.mapTopology.network(7,:) = [0,8,9,4,7]; 
pomdpModel.mapTopology.network(8,:) = [7,0,0,0,8]; 
pomdpModel.mapTopology.network(9,:) = [0,10,11,7,9]; 
pomdpModel.mapTopology.network(10,:) = [9,0,0,0,10]; 
pomdpModel.mapTopology.network(11,:) = [49,0,12,9,11]; 
pomdpModel.mapTopology.network(12,:) = [0,13,14,11,12]; 
pomdpModel.mapTopology.network(13,:) = [12,0,0,0,13]; 
pomdpModel.mapTopology.network(14,:) = [47,0,15,12,14]; 
pomdpModel.mapTopology.network(15,:) = [0,16,17,14,15]; 
pomdpModel.mapTopology.network(16,:) = [15,0,0,0,16]; 
pomdpModel.mapTopology.network(17,:) = [45,18,19,15,17]; 
pomdpModel.mapTopology.network(18,:) = [17,0,0,0,18]; 
pomdpModel.mapTopology.network(19,:) = [0,20,21,17,19]; 
pomdpModel.mapTopology.network(20,:) = [19,0,0,0,20]; 
pomdpModel.mapTopology.network(21,:) = [42,0,22,19,21]; 
pomdpModel.mapTopology.network(22,:) = [0,23,24,21,22]; 
pomdpModel.mapTopology.network(23,:) = [22,0,0,0,23]; 
pomdpModel.mapTopology.network(24,:) = [27,0,25,22,24]; 
pomdpModel.mapTopology.network(25,:) = [26,0,0,24,25]; 
pomdpModel.mapTopology.network(26,:) = [0,25,0,0,26]; 
pomdpModel.mapTopology.network(27,:) = [28,24,0,0,27]; 
pomdpModel.mapTopology.network(28,:) = [29,27,0,40,28]; 
pomdpModel.mapTopology.network(29,:) = [30,28,31,0,29]; 
pomdpModel.mapTopology.network(30,:) = [0,29,0,0,30]; 
pomdpModel.mapTopology.network(31,:) = [32,0,0,29,31]; 
pomdpModel.mapTopology.network(32,:) = [33,31,34,0,32]; 
pomdpModel.mapTopology.network(33,:) = [0,32,0,0,33]; 
pomdpModel.mapTopology.network(34,:) = [35,36,37,32,34]; 
pomdpModel.mapTopology.network(35,:) = [0,34,0,0,35]; 
pomdpModel.mapTopology.network(36,:) = [34,0,0,0,36]; 
pomdpModel.mapTopology.network(37,:) = [38,39,0,34,37]; 
pomdpModel.mapTopology.network(38,:) = [0,37,0,0,38]; 
pomdpModel.mapTopology.network(39,:) = [37,0,0,0,39]; 
pomdpModel.mapTopology.network(40,:) = [41,42,28,43,40]; 
pomdpModel.mapTopology.network(41,:) = [0,40,0,0,41]; 
pomdpModel.mapTopology.network(42,:) = [40,21,0,0,42]; 
pomdpModel.mapTopology.network(43,:) = [44,45,40,46,43]; 
pomdpModel.mapTopology.network(44,:) = [0,43,0,0,44]; 
pomdpModel.mapTopology.network(45,:) = [43,17,0,0,45]; 
pomdpModel.mapTopology.network(46,:) = [0,47,43,48,46]; 
pomdpModel.mapTopology.network(47,:) = [46,14,0,0,47]; 
pomdpModel.mapTopology.network(48,:) = [0,49,46,2,48]; 
pomdpModel.mapTopology.network(49,:) = [48,11,0,0,49]; 

end