function [actions] = sortPromisingActions(pomdp,beliefSpace)
% This function takes the factored belief space and returns the reward 
% function after executing an action a
% -Inputs
% pomdp = the POMDP model read from a file that usually ends with .pomdp, this
%   model is used to extract the translation and the observation
%   probabilities needed for the update.
% beliefSpace = the currently accessible beielf states (factored)
% a = action to be taken
% -Outputs
% actions = array of actions sorted according to most rewarding first

% r(s,a)=\sigma_s'\sigma_o T(s',s,a)O(s',a,o)R(s',s,a)
% PhD Cassandra, page 31

m=length(beliefSpace);
reward = 0;
maxReward = -1000;

for a=1:pomdp.nrActions
    for i=1:m
        s = beliefSpace(i).state;
        pomdp.reward(s,a) = pomdp.transition(:,s,a)'*pomdp.reward3(:,s,a);
    end
end  

for i=1:m
    for a=1:pomdp.nrActions
        s = beliefSpace(i).state;
        reward = beliefSpace(i).value*pomdp.reward(s,a);
        actions(a).action = a;
        actions(a).reward = reward;
    end
end

% I know I know, i had to do the bubble sort myself
notSorted = true;
while notSorted
    notSorted = 0;
    for i=1:length(actions)-1
        if(actions(i).reward<actions(i+1).reward)
            temp = actions(i);
            actions(i) = actions(i+1);
            actions(i+1) = temp;
            notSorted = 1;
        end
    end
end

end