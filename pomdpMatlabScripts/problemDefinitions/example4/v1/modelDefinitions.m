function pomdpModel = modelDefinitions(outputFile,obsDataFile)

pomdpModel.outputFile  = outputFile;
pomdpModel.obsDataFile = obsDataFile;
pomdpModel.discount = 0.95;
pomdpModel.numSpatialStates = 9;
pomdpModel.obsCrossFeature  = 0;
pomdpModel.destCrossFeature = 1;

% define the destinations
pomdpModel.destinations = {'s3d1','s7d2'};

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
pomdpModel.mapTopology.network(1,:) = [0,4,0,0,1]; 
pomdpModel.mapTopology.network(2,:) = [0,5,0,0,2]; 
pomdpModel.mapTopology.network(3,:) = [0,6,0,0,3]; 
pomdpModel.mapTopology.network(4,:) = [1,7,5,0,4]; 
pomdpModel.mapTopology.network(5,:) = [2,8,6,4,5]; 
pomdpModel.mapTopology.network(6,:) = [3,9,0,5,6]; 
pomdpModel.mapTopology.network(7,:) = [4,0,0,0,7]; 
pomdpModel.mapTopology.network(8,:) = [5,0,0,0,8]; 
pomdpModel.mapTopology.network(9,:) = [6,0,0,0,9];


% % Manually Build The Observation Probabilities
% 
% fprintf(fid,'\n\n# O : <action> : <end-state> : <observation> %%f');
% fprintf(fid,'\nO: * : * : * 0.0');
% 
% fprintf(fid,'\nO: * : s1d1 : Down 0.9');
% fprintf(fid,'\nO: * : s1d1 : Nothing 0.1');
% fprintf(fid,'\nO: * : s1d2 : Down 0.9');
% fprintf(fid,'\nO: * : s1d2 : Nothing 0.1');
% 
% fprintf(fid,'\nO: * : s2d1 : Down 0.9');
% fprintf(fid,'\nO: * : s2d1 : Nothing 0.1');
% fprintf(fid,'\nO: * : s2d2 : Down 0.9');
% fprintf(fid,'\nO: * : s2d2 : Nothing 0.1');
% 
% fprintf(fid,'\nO: * : s3d1 : Down 0.1');
% fprintf(fid,'\nO: * : s3d1 : Nothing 0.9');
% fprintf(fid,'\nO: * : s3d2 : Down 0.9');
% fprintf(fid,'\nO: * : s3d2 : Nothing 0.1');
% 
% fprintf(fid,'\nO: * : s4d1 : Right 0.9');
% fprintf(fid,'\nO: * : s4d1 : Nothing 0.1');
% fprintf(fid,'\nO: * : s4d2 : Down 0.9');
% fprintf(fid,'\nO: * : s4d2 : Nothing 0.1');
% 
% fprintf(fid,'\nO: * : s5d1 : Right 0.9');
% fprintf(fid,'\nO: * : s5d1 : Nothing 0.1');
% fprintf(fid,'\nO: * : s5d2 : Left 0.9');
% fprintf(fid,'\nO: * : s5d2 : Nothing 0.1');
% 
% fprintf(fid,'\nO: * : s6d1 : Up 0.9');
% fprintf(fid,'\nO: * : s6d1 : Nothing 0.1');
% fprintf(fid,'\nO: * : s6d2 : Left 0.9');
% fprintf(fid,'\nO: * : s6d2 : Nothing 0.1');
% 
% fprintf(fid,'\nO: * : s7d1 : Up 0.9');
% fprintf(fid,'\nO: * : s7d1 : Nothing 0.1');
% fprintf(fid,'\nO: * : s7d2 : Up 0.1');
% fprintf(fid,'\nO: * : s7d2 : Nothing 0.9');
% 
% fprintf(fid,'\nO: * : s8d1 : Up 0.9');
% fprintf(fid,'\nO: * : s8d1 : Nothing 0.1');
% fprintf(fid,'\nO: * : s8d2 : Up 0.9');
% fprintf(fid,'\nO: * : s8d2 : Nothing 0.1');
% 
% fprintf(fid,'\nO: * : s9d1 : Up 0.9');
% fprintf(fid,'\nO: * : s9d1 : Nothing 0.1');
% fprintf(fid,'\nO: * : s9d2 : Up 0.9');
% fprintf(fid,'\nO: * : s9d2 : Nothing 0.1');
% % for i=1:length(destinations)
% %     for j=1:numSpacialStates
% %         index = (i-1)*numSpacialStates + j;   
% %         % Find the number of reachable nodes (connected)
% %         n = length(find(mapTopology.network(index,:)));   
% %         if length(find(observationsUncertainty)) ~=0
% %             observationProb = 1/n;
% %         else
% %             observationProb = 1;
% %         end
% %         %Check the connections
% %         for s=1:length(actions)
% %             if (mapTopology.network(index,s)~=0)
% %                 fprintf(fid,'\nO: * : s%dd%d : %s %f',j,i,observations{s},observationProb);
% %                 if length(find(observationsUncertainty)) == 0
% %                     break;
% %                 end
% %             end
% %         end
% %         fprintf(fid,'\n');    
% %     end
% % end
end