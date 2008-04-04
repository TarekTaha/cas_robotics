function [factoredBelief] = factorStateSpace(pomdp,currentBelief)
% This function takes the belief space and returns only the 
% accessible states (belief!=0)
% -Inputs
% pomdp = the POMDP model read from a file that usually ends with .pomdp, this
%   model is used to extract the translation and the observation
%   probabilities needed for the update.
% currentBelief = the current belief vector of length (1,n) where n is the
%   number of states in the POMDP model.
% -Outputs
% subStates = the current accessible states
n = pomdp.nrStates;
m=0;
for i=1:n
    if currentBelief(i) ~= 0
        m = m+1;
        factoredBelief(m).state = i;
        factoredBelief(m).value = currentBelief(i);
    end
end