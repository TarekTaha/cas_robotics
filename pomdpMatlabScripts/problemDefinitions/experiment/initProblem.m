function initProblem()
outputModelFile = 'paperexperiment.pomdp';
obsDataFile     = 'exp_tasks_list';
pomdpModel = modelDefinitions(outputModelFile,obsDataFile);
generatePomdpModel(pomdpModel);
display('POMDP Model Generated');
end