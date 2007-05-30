function pomdpModel = modelDefinitions(outputFile,obsDataFile)

pomdpModel.outputFile  = outputFile;
pomdpModel.obsDataFile = obsDataFile;

global mapTopology;
pomdpModel.discount = 0.95;
pomdpModel.numSpatialStates = 49;

% define the destinations
pomdpModel.destinations = {'s1d1','s6d2','s26d3','s30d4'};

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

pomdpModel.mapTopology.nnodes = pomdpModel.numSpatialStates*length(pomdpModel.destinations);
pomdpModel.mapTopology.network = zeros(mapTopology.nnodes,length(pomdpModel.actions));
    
for i=0:(length(pomdpModel.destinations)-1)
    pomdpModel.mapTopology.network(1+i*pomdpModel.numSpatialStates,:) = [0,2,0,0,1]; 
    pomdpModel.mapTopology.network(2+i*pomdpModel.numSpatialStates,:) = [1,3,48,0,2]; 
    pomdpModel.mapTopology.network(3+i*pomdpModel.numSpatialStates,:) = [2,4,0,0,3]; 
    pomdpModel.mapTopology.network(4+i*pomdpModel.numSpatialStates,:) = [3,0,7,5,4]; 
    pomdpModel.mapTopology.network(5+i*pomdpModel.numSpatialStates,:) = [0,6,4,0,5]; 
    pomdpModel.mapTopology.network(6+i*pomdpModel.numSpatialStates,:) = [5,0,0,0,6]; 
    pomdpModel.mapTopology.network(7+i*pomdpModel.numSpatialStates,:) = [0,8,9,4,7]; 
    pomdpModel.mapTopology.network(8+i*pomdpModel.numSpatialStates,:) = [7,0,0,0,8]; 
    pomdpModel.mapTopology.network(9+i*pomdpModel.numSpatialStates,:) = [0,10,11,7,9]; 
    pomdpModel.mapTopology.network(10+i*pomdpModel.numSpatialStates,:) = [9,0,0,0,10]; 
    pomdpModel.mapTopology.network(11+i*pomdpModel.numSpatialStates,:) = [49,0,12,9,11]; 
    pomdpModel.mapTopology.network(12+i*pomdpModel.numSpatialStates,:) = [0,13,14,11,12]; 
    pomdpModel.mapTopology.network(13+i*pomdpModel.numSpatialStates,:) = [12,0,0,0,13]; 
    pomdpModel.mapTopology.network(14+i*pomdpModel.numSpatialStates,:) = [47,0,15,12,14]; 
    pomdpModel.mapTopology.network(15+i*pomdpModel.numSpatialStates,:) = [0,16,17,14,15]; 
    pomdpModel.mapTopology.network(16+i*pomdpModel.numSpatialStates,:) = [15,0,0,0,16]; 
    pomdpModel.mapTopology.network(17+i*pomdpModel.numSpatialStates,:) = [45,18,19,15,17]; 
    pomdpModel.mapTopology.network(18+i*pomdpModel.numSpatialStates,:) = [17,0,0,0,18]; 
    pomdpModel.mapTopology.network(19+i*pomdpModel.numSpatialStates,:) = [0,20,21,17,19]; 
    pomdpModel.mapTopology.network(20+i*pomdpModel.numSpatialStates,:) = [19,0,0,0,20]; 
    pomdpModel.mapTopology.network(21+i*pomdpModel.numSpatialStates,:) = [42,0,22,19,21]; 
    pomdpModel.mapTopology.network(22+i*pomdpModel.numSpatialStates,:) = [0,23,24,21,22]; 
    pomdpModel.mapTopology.network(23+i*pomdpModel.numSpatialStates,:) = [22,0,0,0,23]; 
    pomdpModel.mapTopology.network(24+i*pomdpModel.numSpatialStates,:) = [27,0,25,22,24]; 
    pomdpModel.mapTopology.network(25+i*pomdpModel.numSpatialStates,:) = [26,0,0,24,25]; 
    pomdpModel.mapTopology.network(26+i*pomdpModel.numSpatialStates,:) = [0,25,0,0,26]; 
    pomdpModel.mapTopology.network(27+i*pomdpModel.numSpatialStates,:) = [28,24,0,0,27]; 
    pomdpModel.mapTopology.network(28+i*pomdpModel.numSpatialStates,:) = [29,27,0,40,28]; 
    pomdpModel.mapTopology.network(29+i*pomdpModel.numSpatialStates,:) = [30,28,31,0,29]; 
    pomdpModel.mapTopology.network(30+i*pomdpModel.numSpatialStates,:) = [0,29,0,0,30]; 
    pomdpModel.mapTopology.network(31+i*pomdpModel.numSpatialStates,:) = [32,0,0,29,31]; 
    pomdpModel.mapTopology.network(32+i*pomdpModel.numSpatialStates,:) = [33,31,34,0,32]; 
    pomdpModel.mapTopology.network(33+i*pomdpModel.numSpatialStates,:) = [0,32,0,0,33]; 
    pomdpModel.mapTopology.network(34+i*pomdpModel.numSpatialStates,:) = [35,36,37,32,34]; 
    pomdpModel.mapTopology.network(35+i*pomdpModel.numSpatialStates,:) = [0,34,0,0,35]; 
    pomdpModel.mapTopology.network(36+i*pomdpModel.numSpatialStates,:) = [34,0,0,0,36]; 
    pomdpModel.mapTopology.network(37+i*pomdpModel.numSpatialStates,:) = [38,39,0,34,37]; 
    pomdpModel.mapTopology.network(38+i*pomdpModel.numSpatialStates,:) = [0,37,0,0,38]; 
    pomdpModel.mapTopology.network(39+i*pomdpModel.numSpatialStates,:) = [37,0,0,0,39]; 
    pomdpModel.mapTopology.network(40+i*pomdpModel.numSpatialStates,:) = [41,42,28,43,40]; 
    pomdpModel.mapTopology.network(41+i*pomdpModel.numSpatialStates,:) = [0,40,0,0,41]; 
    pomdpModel.mapTopology.network(42+i*pomdpModel.numSpatialStates,:) = [40,21,0,0,42]; 
    pomdpModel.mapTopology.network(43+i*pomdpModel.numSpatialStates,:) = [44,45,40,46,43]; 
    pomdpModel.mapTopology.network(44+i*pomdpModel.numSpatialStates,:) = [0,43,0,0,44]; 
    pomdpModel.mapTopology.network(45+i*pomdpModel.numSpatialStates,:) = [43,17,0,0,45]; 
    pomdpModel.mapTopology.network(46+i*pomdpModel.numSpatialStates,:) = [0,47,43,48,46]; 
    pomdpModel.mapTopology.network(47+i*pomdpModel.numSpatialStates,:) = [46,14,0,0,47]; 
    pomdpModel.mapTopology.network(48+i*pomdpModel.numSpatialStates,:) = [0,49,46,2,48]; 
    pomdpModel.mapTopology.network(49+i*pomdpModel.numSpatialStates,:) = [48,11,0,0,49]; 
end

end