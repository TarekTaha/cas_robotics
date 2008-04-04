function updatedBelief = updateBelief(pomdp,prevBelief,currentObs,prevAction)
% This function updates the beliefs given a current observation and the
% last action taken.
% Parameters:
% -Inputs
% pomdp = the POMDP model read from a file that usually ends with .pomdp, this
%   model is used to extract the translation and the observation
%   probabilities needed for the update.
% prevBelief = the current belief vector of length (1,n) where n is the
%   number of states in the POMDP model.
% currentObs = the current observation obtained.
% prevAction = the last action taken to come to the current state.
% -Outputs
% updatedBelief = a belief vector of length (1,n) with the updates beliefs.

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
n = pomdp.nrStates;

updatedBelief = zeros(1,n);

for i=1:n % next state
    for j=1:n % current 
        if prevBelief(i)*pomdp.transition(i,j,prevAction) ~=0
            %display('YES');
        end
        updatedBelief(i) = updatedBelief(i) + prevBelief(i)*pomdp.transition(i,j,prevAction)*pomdp.observation(i,prevAction,currentObs);
    end
end

% Normalize probabilites
denom = 0;
for i=1:n
    denom = denom +  updatedBelief(i);
end

if denom ~= 0
    for i=1:n
        updatedBelief(i) = updatedBelief(i)/denom;
    end
end

end