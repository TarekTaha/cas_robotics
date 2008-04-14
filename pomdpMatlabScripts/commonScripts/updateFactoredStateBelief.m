function totalBelief = updateFactoredStateBelief(pomdp,factoredBelief,currentObs,prevAction)
% This function updates the factored belief given a current observation and
% the last action taken.
% Parameters:
% -Inputs
% pomdp = the POMDP model read from a file that usually ends with .pomdp, this
%   model is used to extract the translation and the observation
%   probabilities needed for the update.
% factoredBelief = the current belief vector of length (1,n) where n is the
%   number of states in the POMDP model.
% currentObs = the current observation obtained.
% prevAction = the last action taken to come to the current state.
% -Outputs
% updatedFactoredBelief = a belief vector of length (1,n) with the updated
% factored beliefs.

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


reachableStates = getReachableStates(pomdp,factoredBelief,currentObs,prevAction);
n = length(reachableStates);
m = length(factoredBelief);
totalBelief =0;
for i=1:n % next state
    sp = reachableStates(i).state;
    for j=1:m % current
        s = factoredBelief(j).state;
        totalBelief = totalBelief + factoredBelief(j).value*pomdp.transition(sp,s,prevAction)*pomdp.observation(sp,prevAction,currentObs);
    end
end

end