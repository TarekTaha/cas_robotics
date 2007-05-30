function action = findPolicyAction(policy,belief)
% given a parsed policy file, this function will get you the best action
% based on the policy and the current belief

numBeliefs = length(belief);
expectedReward = zeros(1,policy.numPlanes);

for i=1:policy.numPlanes
    for j=1:numBeliefs
        % check if this vector is missing the relevant information
        if isempty(policy.planeEntries{i,j}) && belief(j)~=0
            % large negative number to insure that this vector will not be
            % picked up later
            expectedReward(i) = - 10000; 
            break;
        else
            if ~isempty(policy.planeEntries{i,j})
                expectedReward(i) = expectedReward(i) + belief(j)*policy.planeEntries{i,j};
            end
        end
    end
end

[maxValue,index]  = max(expectedReward);
action = policy.planeActions(index) + 1; % Actions in the output Policy start from 0

end