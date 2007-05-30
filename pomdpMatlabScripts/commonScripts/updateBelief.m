function updatedBelief = updateBelief(pomdp,prevBelief,currentObs,prevAction)
% This function updates the beliefs given a current observation and the
% last action taken.
% Parameters:
% -Inputs
% pomdp = the POMDP model read from a file that usually ends with .pomdp, this
%   model is used to extract the translation and the observation
%   probabilities needed for the update.
% prevBelief = the current belief vector of length (1,n) where n is the
%   number of states in the POMDP model.
% currentObs = the current observation obtained.
% prevAction = the last action taken to come to the current state.
% -Outputs
% updatedBelief = a belief vector of length (1,n) with the updates beliefs.

% get the number of states from the model
n = pomdp.nrStates;

updatedBelief = zeros(1,n);

for i=1:n % next state
    for j=1:n % current 
        if prevBelief(i)*pomdp.transition(i,j,prevAction) ~=0
            %display('YES');
        end
        updatedBelief(i) = updatedBelief(i) + prevBelief(i)*pomdp.transition(i,j,prevAction)*pomdp.observation(i,prevAction,currentObs);
    end
end

% Normalize probabilites
denom = 0;
for i=1:n
    denom = denom +  updatedBelief(i);
end

if denom ~= 0
    for i=1:n
        updatedBelief(i) = updatedBelief(i)/denom;
    end
end

end