function action = findPolicyAction(policy,belief)
% given a parsed policy file, this function will get you the best action
% based on the policy and the current belief

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