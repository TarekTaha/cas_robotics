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

%reachableStates(100).state = 0;
%reachableStates(100).value = 0;
reachableStates = [];
n = length(currnetfactoredBelief);
k = 0;
tic;
for i=1:n
    if currnetfactoredBelief(i).value == 0
        continue;
    end
    s = currnetfactoredBelief(i).state;
    m = find(pomdp.transition(:,s,a));
     for sp=1:length(m)
         alreadyExist = 0;
        % Now, proceed with this example, eliminating redundant elements of a vector. Note that once a vector is sorted, any redundant elements are adjacent. In addition, any equal adjacent elements in a vector create a zero entry in the DIFF of that vector. This suggests the following implementation for the operation. You are attempting to select the elements of the sorted vector that correspond to nonzero differences.
        % 
        % % First try. NOT QUITE RIGHT!! 
        % x = sort(x(:)); 
        % difference = diff(x); 
        % y = x(difference~=0);
        % 
        % This is almost correct, but you have forgotten to take into account the fact that the DIFF function returns a vector that has one fewer element than the input vector. In your first algorithm, the last unique element is not accounted for. You can fix this by adding one element to the vector x before taking the difference. You need to make sure that the added element is always different than the previous element. One way to do this is to add a NaN.
        % 
        % % Final version. 
        % x = sort(x(:)); 
        % difference = diff([x;NaN]); 
        % y = x(difference~=0);
        % or use unique function
         if(pomdp.observation(sp,a,o))
             for r=1:k%length(reachableStates)
                if reachableStates(r).state == m(sp)
                    alreadyExist = 1;
                    break;
                end
             end
             if  ~alreadyExist
                k = k+1;
                reachableStates(k).state = m(sp);
                reachableStates(k).value = 0;                
             end
         end
     end
end


%reachableStates = cell2mat(reachableStates);
reachableStates = reachableStates(1:k);
toc;
% x = sort(x(:)); 
        % difference = diff([x;NaN]); 
        % y = x(difference~=0);
% reachableStates = unique(reachableStates);
end