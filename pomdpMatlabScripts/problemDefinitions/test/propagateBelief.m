function [newBelief] = propagateBelief(belief,currentState)

newBelief = zeros(1,length(belief));

for i=1:length(belief)
    if(belief(1,i)~=0)
        newBelief(1,currentState + floor(i/49)*49) = belief(1,i);
    end
end

end