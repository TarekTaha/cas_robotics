function [maxValue action] = RTBSS(pomdp,currentBelief,d,D)
% This function implements the Real-Time Belief State Space 
% where it calculates in real-time the estimated Value function
% upto a certain depth Depth d
% -Inputs
% pomdp = the POMDP model read from a file that usually ends with .pomdp, this
%   model is used to extract the translation and the observation
%   probabilities needed for the update.
% currentBelief = the current belief vector of length (1,n) where n is the
%   number of states in the POMDP model.
% d = the depth of the search tree to be used
% -Outputs
% maxValue = the maximium value function obtain while searching through
% that depth
% action = the action that generates the maximium value function

% ************************************************************************\
% * Copyright (C) 2007 - 2008 by:                                         *
% *    Tarek Taha, Jaime Valls Miro, Gamini Dissanayake                   *
% *     CAS-UTS  {t.taha,j.vallsmiro,g.dissanayake}@cas.edu.au            *
% *                                                                       *
% *                                                                       *
% * This program is free software; you can redistribute it and/or modify  *
% * it under the terms of the GNU General Public License as published by  *
% * the Free Software Foundation; either version 2 of the License, or     *
% * (at your option) any later version.                                   *
% *                                                                       *
% * This program is distributed in the hope that it will be useful,       *
% * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
% * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
% * GNU General Public License for more details.                          *
% *                                                                       *
% * You should have received a copy of the GNU General Public License     *
% * along with this program; if not, write to the                         *
% * Free Software Foundation, Inc.,                                       *
% * 51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
% ************************************************************************/

% get the number of states from the model
if d == 0
    maxValue = getUtilityFunctionValue(pomdp,currentBelief);
%     addstr = '';
%     for j=1:(D-d)
%         addstr = strcat(addstr,'    -');
%     end    
%     addstr = strcat(addstr,'>');
%     display(sprintf('%s Utility function''s Value:=%f',addstr,maxValue));
%     display(sprintf('%s Belief at Root with depth:%d is S1:%f S2:%f',addstr,d,currentBelief(1).value,currentBelief(2).value));
    return 
end

maxValue = -1000;

actions = sortPromisingActions(pomdp,currentBelief);
n = length(actions);

for i=1:n 
    %Get the action from the sorted actions list
    a = actions(i).action;
    currReward = rewardB(pomdp,currentBelief,a);
    upperBound = currReward + getHeuristicValue(pomdp,currentBelief,a,d);
    %display(sprintf('Upper Bound =:%f Current Reward:=%f Heuristic:=%f',upperBound,currReward,getHeuristicValue(pomdp,currentBelief,a,d)));
    if upperBound > maxValue
        for o=1:pomdp.nrObservations
%             addstr = '';
%             for j=1:(D-d)
%                 addstr = strcat(addstr,'    =');
%             end
%             addstr = strcat(addstr,'>'); 
            %display(sprintf('%sDepth is:%d  Action is:%d Obs:=%d Belief S1:=%f S2:=%f',addstr,d,a,o,currentBelief(1).value,currentBelief(2).value));            
            currReward = currReward + pomdp.gamma*updateFactoredStateBelief(pomdp,currentBelief,o,a)*RTBSS(pomdp,updateFactoredBelief(pomdp,currentBelief,o,a),d-1,D);
        end
        %display(sprintf('Actual CurrentReward :=%f Max:=%f',currReward,maxValue));
        if currReward >maxValue
            maxValue = currReward;
            if d == D
                action = a;
            end
        end
    end
end

end