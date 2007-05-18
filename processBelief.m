function [maxValue valueFunctions action index] = processBelief(beliefs)

filename = '/home/tataha/pomdp/examples/solution.alpha';

[actions vectors] = loadSolution(filename);
valueFunctions = zeros(size(actions));

for i=1:length(actions)
    for j=1:length(beliefs)
        valueFunctions(i) = valueFunctions(i) + beliefs(j)*vectors(i,j);
    end
end

[maxValue,index]  = max(valueFunctions);
action = actions(index);
end
