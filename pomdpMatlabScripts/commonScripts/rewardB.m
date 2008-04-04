function [reward] = rewardB(pomdp,beliefSpace,action)
% This function takes the factored belief space and returns the reward 
% function after executing an action a
% -Inputs
% pomdp = the POMDP model read from a file that usually ends with .pomdp, this
%   model is used to extract the translation and the observation
%   probabilities needed for the update.
% beliefSpace = the currently accessible beielf states (factored)
% a = action to be taken
% -Outputs
% reward = the immediate reward on that belief after taking the action

% r(s,a)=\sigma_s'\sigma_o T(s',s,a)O(s',a,o)R(s',s,a)
% PhD Cassandra, page 31

m=length(beliefSpace);
reward = 0;

% for a=1:pomdp.nrActions
%     for i=1:m
%         s = beliefSpace(i).state;
%         pomdp.reward(s,a) = pomdp.transition(:,s,a)'*pomdp.reward3(:,s,a);
%     end
% end

for i=1:m
    s = beliefSpace(i).state;
    reward = reward + beliefSpace(i).value*pomdp.reward(s,action);
end

end