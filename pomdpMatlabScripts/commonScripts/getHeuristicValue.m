function [heuristicValue] = getHeuristicValue(pomdp,factoredBelief,action,depth)
% This function takes the factored belief space and returns the reward 
% function after executing an action a
% -Inputs
% pomdp = the POMDP model read from a file that usually ends with .pomdp, this
%   model is used to extract the translation and the observation
%   probabilities needed for the update.
% factoredBelief = the currently accessible beielf states (factored)
% -Outputs
% reward = the max immediate reward on that belief

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

heuristicValue = 0;
%maxR = max(max(pomdp.reward(:,:)));
for i=1:depth
    maxR = -10000;
    for j=1:length(factoredBelief)
        s = factoredBelief(j).state;
        temp = pomdp.reward(s,action);
        if temp > maxR
            maxR = temp;
        end
    end
    heuristicValue = heuristicValue + pomdp.gamma.^i*maxR;
end

end