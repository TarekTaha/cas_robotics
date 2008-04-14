function [action maxValue] = findPolicyAction(policy,belief)
% This function takes a parsed policy file as input and the current belief
% and it returns the best action to execute with the value corresponding to
% it.
% -Inputs:
% policy = the policy  parsed from a zmdp solution file (using zmdpPolicy 
%    Parser function).
% belief = the current belief vector of length (1,n) where n is the
%   number of states in the POMDP model.
% -Output:
% maxValue = the maximium reward value associated with the best action
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

expectedReward = zeros(1,policy.numPlanes);

for i=1:policy.numPlanes
      expectedReward(i) =  sum(belief.*cell2mat(policy.planeEntries(i,:)));
end

[maxValue,index]  = max(expectedReward);
action = policy.planeActions(index) + 1; % Actions in the output Policy start from 0

end