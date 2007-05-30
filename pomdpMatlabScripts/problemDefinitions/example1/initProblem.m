function initProblem()
outputModelFile = 'example4v1.pomdp';
obsDataFile     = '/home/tataha/Matlab Scripts/pomdpMatlabScripts/problemDefinitions/trainingData/tasks_list';
pomdpModel = modelDefinitions(outputModelFile,obsDataFile);
generatePomdpModel(pomdpModel);
display('POMDP Model Generated');
end