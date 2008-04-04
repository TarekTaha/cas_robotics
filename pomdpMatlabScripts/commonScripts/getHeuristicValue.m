function [heuristicValue] = getHeuristicValue(pomdp,factoredBelief,action,depth)
% This function takes the factored belief space and returns the reward 
% function after executing an action a
% -Inputs
% pomdp = the POMDP model read from a file that usually ends with .pomdp, this
%   model is used to extract the translation and the observation
%   probabilities needed for the update.
% factoredBelief = the currently accessible beielf states (factored)
% -Outputs
% reward = the max immediate reward on that belief

heuristicValue = 0;
m = length(factoredBelief);

maxR = -10000;
for i=1:m
    localMax = max(max(pomdp.reward3(:,factoredBelief(i).state,:)));
    if  localMax> maxR
        maxR = localMax;
    end
end

for i=1:depth
    heuristicValue = heuristicValue + pomdp.gamma.^i*maxR;
end

end