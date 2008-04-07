function initProblem()
outputModelFile = 'paperexperiment.pomdp';
%outputModelFile = 'minimal_interaction.pomdp';
obsDataFile     = 'exp_tasks_list_large';
%obsDataFile     = 'minimal_interaction_tasks';
pomdpModel = modelDefinitions(outputModelFile,obsDataFile);
generatePomdpModel(pomdpModel);
display('POMDP Model Generated');
end