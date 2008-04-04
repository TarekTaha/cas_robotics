function [maxRewardValue] = getUtilityFunctionValue(pomdp,factoredBelief)
% This function takes the factored belief space and returns the reward 
% function after executing an action a
% -Inputs
% pomdp = the POMDP model read from a file that usually ends with .pomdp, this
%   model is used to extract the translation and the observation
%   probabilities needed for the update.
% factoredBelief = the currently accessible beielf states (factored)
% -Outputs
% reward = the max immediate reward on that belief

maxRewardValue = 0;

for a=1:pomdp.nrActions
    reward = rewardB(pomdp,factoredBelief,a);
    if reward > maxRewardValue
        maxRewardValue = reward;
    end
end

end
  