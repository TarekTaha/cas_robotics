function initProblem()
outputModelFile = 'paperexperiment_mod.pomdp';
obsDataFile     = 'exp_tasks_list_large_sub';
pomdpModel = modelDefinitions(outputModelFile,obsDataFile);
generatePomdpModel(pomdpModel);
display('POMDP Model Generated');
end