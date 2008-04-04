function [reachableStates] = getReachableStates(pomdp,currnetfactoredBelief,o,a)
% This function takes the belief space and returns only the 
% possible successor states
% -Inputs
% pomdp = the POMDP model read from a file that usually ends with .pomdp, this
%   model is used to extract the translation and the observation
%   probabilities needed for the update.
% currentBelief = the current belief vector of length (1,n) where n is the
%   number of states in the POMDP model.
% a = action performed
% o = observation recieved
% -Outputs
% reachableStates = the next accessible states

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

n = length(currnetfactoredBelief);
m = pomdp.nrStates;
k = 0;
for i=1:n
    s = currnetfactoredBelief(i).state;
    for j=1:m
        alreadyExist = 0;
        if pomdp.transition(j,s,a) ~= 0 
            if pomdp.observation(j,a,o) ~= 0
                if exist('reachableStates')
                    for s=1:length(reachableStates)
                        if reachableStates(s).state == j
                            alreadyExist = 1;
                            break;
                        end
                    end
                end
                if ~alreadyExist
                    k = k+1;
                    reachableStates(k).state = j;
                    reachableStates(k).value = 0;
                end
            end
        end
    end
end