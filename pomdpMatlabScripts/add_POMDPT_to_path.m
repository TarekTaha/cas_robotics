global POMDPT_HOME
POMDPT_HOME = pwd;
if (~exist ('POMDPT_HOME'))
    error ('Please define the global POMDPT_HOME before running add_POMDPT_to_path.');
end

addpath(genpath(POMDPT_HOME));

folders = {'commonScripts'};
addpath(POMDPT_HOME);

for i=1:length(folders)
  addpath(fullfile(POMDPT_HOME, folders{i}))
end