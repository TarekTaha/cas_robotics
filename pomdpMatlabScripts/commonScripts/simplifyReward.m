function simplifyReward 
% simplifyReward - compute R(s,a) from R(s,a,s')

% Author: Matthijs Spaan
% $Id: simplifyReward.m,v 1.5 2004/07/13 14:01:00 mtjspaan Exp $
% Copyright (c) 2003,2004 Universiteit van Amsterdam.  All rights reserved.
% More information is in the file named COPYING.

% r(s,a)=\sigma_s'\sigma_o T(s',s,a)O(s',a,o)R(s',s,a)
% PhD Cassandra, page 31

global pomdp;

%pomdp.reward = zeros(pomdp.nrStates,pomdp.nrActions);
 
  for a=1:pomdp.nrActions
    for s=1:pomdp.nrStates
      pomdp.reward(s,a) = pomdp.transition(:,s,a)'*pomdp.reward3(:,s,a);
    end
  end
 
end
