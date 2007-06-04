function initProblem()
outputModelFile = 'example3.pomdp';
obsDataFile     = '/home/tataha/Matlab Scripts/pomdpMatlabScripts/problemDefinitions/trainingData/tasks_list2';
pomdpModel = modelDefinitions(outputModelFile,obsDataFile);
generatePomdpModel(pomdpModel);
display('POMDP Model Generated');
end