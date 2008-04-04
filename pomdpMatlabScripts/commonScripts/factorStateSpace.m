function [factoredBelief] = factorStateSpace(pomdp,currentBelief)
% This function takes the belief space and returns only the 
% accessible states (belief!=0)
% -Inputs
% pomdp = the POMDP model read from a file that usually ends with .pomdp, this
%   model is used to extract the translation and the observation
%   probabilities needed for the update.
% currentBelief = the current belief vector of length (1,n) where n is the
%   number of states in the POMDP model.
% -Outputs
% subStates = the current accessible states

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

n = pomdp.nrStates;
m=0;
for i=1:n
    if currentBelief(i) ~= 0
        m = m+1;
        factoredBelief(m).state = i;
        factoredBelief(m).value = currentBelief(i);
    end
end