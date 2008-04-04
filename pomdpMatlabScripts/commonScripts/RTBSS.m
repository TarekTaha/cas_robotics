function [maxValue action] = RTBSS(pomdp,currentBelief,d)
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
D = 2;
if d == 0
    maxValue = getUtilityFunctionValue(pomdp,currentBelief);
    %display(sprintf('U:=%f',maxValue));
    return 
end

maxValue = -1000;

actions = sortPromisingActions(pomdp,currentBelief);
n = length(actions);

%display(sprintf('Belief in s1:=%f in s2:=%f',currentBelief(1).value,currentBelief(2).value));

for i=1:n 
    %Get the action from the sorted actions list
    a = actions(i).action;
    currReward = rewardB(pomdp,currentBelief,a);
    upperBound = currReward + getHeuristicValue(pomdp,currentBelief,a,d);
    if upperBound > maxValue
        for o=1:pomdp.nrObservations
            currReward = currReward + pomdp.gamma*updateFactoredStateBelief(pomdp,currentBelief,o,a)*RTBSS(pomdp,updateFactoredBelief(pomdp,currentBelief,o,a),d-1);
        end
        if currReward >maxValue
            maxValue = currReward;
            if d == D
                action = a;
            end
        end
    end
end

end