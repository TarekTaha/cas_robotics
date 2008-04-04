function [newBelief action] = testPolicy(pomdpModelFile,policyFile,initialBelief,currentState)
pomdpModel = readPOMDP(pomdpModelFile);
policy = zmdpPolicyParse(policyFile);

currentBelief = zeros(1,294);
for i=1:length(initialBelief)
    currentBelief(1,currentState+(i-1)*49) = initialBelief(1,i);
end
obsIndx    = 5; % NoInput
prevAction = 4; % Nothing
run = true;
while (run)
    reply = input('Enter Your Observation [Arrows or KeyPad 5]:', 's');
    if isempty(reply)
        continue;
    end
    switch (reply)
        case '8'
            obsIndx = 1;
        case '2'
            obsIndx = 2;
        case '6'
            obsIndx = 3;
        case '4'
            obsIndx = 4;
        case '5'
            obsIndx = 5;
        case 'q'
            run = false;
        otherwise
           display('Unknown Input');
           break;
    end
        display(sprintf('Observation is:%d',obsIndx));
        % Update the Belief
        updatedBelief = updateBelief(pomdpModel,currentBelief,obsIndx,prevAction);
        [value , mostProbableCurrentLocation] = max(updatedBelief);
        % Determine the action
        action = findPolicyAction(policy,updatedBelief);
        prevAction = action; 
        % See where this action will take us
        [value, index] = max(pomdpModel.transition(:,mostProbableCurrentLocation,action));
        x = mod(index,49);
        currentBelief=propagateBelief(updatedBelief,x);
        for i=1:length(currentBelief)
            if(currentBelief(1,i)~=0)
                display(sprintf('Destination[%d] Belief:=%f CurrentState:=%d Action:=%d',floor(i/49),currentBelief(1,i),x,action));
            end
        end
end

end