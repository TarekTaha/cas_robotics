function initProblem()
outputModelFile = 'example1.pomdp';
obsDataFile     = '../trainingData/tasks_list';
pomdpModel = modelDefinitions(outputModelFile,obsDataFile);
generatePomdpModel(pomdpModel);
display('POMDP Model Generated');
end