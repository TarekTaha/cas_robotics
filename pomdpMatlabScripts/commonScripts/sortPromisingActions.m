function [actions] = sortPromisingActions(pomdp,beliefSpace)
% This function takes the factored belief space and returns the reward 
% function after executing an action a
% -Inputs
% pomdp = the POMDP model read from a file that usually ends with .pomdp, this
%   model is used to extract the translation and the observation
%   probabilities needed for the update.
% beliefSpace = the currently accessible beielf states (factored)
% a = action to be taken
% -Outputs
% actions = array of actions sorted according to most rewarding first

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


m=length(beliefSpace);
reward = 0;
maxReward = -1000;

for a=1:pomdp.nrActions
    for i=1:m
        s = beliefSpace(i).state;
        pomdp.reward(s,a) = pomdp.transition(:,s,a)'*pomdp.reward3(:,s,a);
    end
end  

for i=1:m
    for a=1:pomdp.nrActions
        s = beliefSpace(i).state;
        reward = beliefSpace(i).value*pomdp.reward(s,a);
        actions(a).action = a;
        actions(a).reward = reward;
    end
end

% I know I know, i had to do the bubble sort myself
notSorted = true;
while notSorted
    notSorted = 0;
    for i=1:length(actions)-1
        if(actions(i).reward<actions(i+1).reward)
            temp = actions(i);
            actions(i) = actions(i+1);
            actions(i+1) = temp;
            notSorted = 1;
        end
    end
end

end