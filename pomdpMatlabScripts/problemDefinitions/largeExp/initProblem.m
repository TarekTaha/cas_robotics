function initProblem()
outputModelFile = 'paperexperiment.pomdp';
obsDataFile     = 'exp_tasks_list_large';
pomdpModel = modelDefinitions(outputModelFile,obsDataFile);
generatePomdpModel(pomdpModel);
display('POMDP Model Generated');
end