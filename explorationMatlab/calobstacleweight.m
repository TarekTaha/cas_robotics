%% calobstacleweight
%
% *Description:* this function calculated the weighted amount of obstacle info.
% it uses the GetImpLevInfo() function to break up known points into
% their areas

%% Function Call
%
% *Inputs:* NULL
%
% *Returns:* NULL

function obstacleweight = calobstacleweight()

%% Variables
global workspace

%% Get obstacle point matrices P, in Lev 1->3
% $$P_1:=(x_j,y_j,z_j)_{j=1,...,m_1}, P_2:=(x_j,y_j,z_j)_{j=1,...,m_2},
% P_3:=(x_j,y_j,z_j)_{j=1,...,m_3}$$

%%
% $$P_1\in P_2\in P_3$$ 

% Remember level 1 is inside level 2 and 3 and level 2 is inside level 3
[level1_ob,level2_ob,level3_ob]=GetImpLevInfo(workspace.indexedobsticles);

%% Multiply by weighting
% $$W_{ob}=C_{dot1}\times m_1 + C_{dot2}\times m_2 + C_{dot3}\times
% m_3$$

% This is the difference in weighted unknown points
obstacleweight=(size(level1_ob,1)*workspace.dotweight(1)+...
                size(level2_ob,2)*workspace.dotweight(2)+...
                size(level3_ob,3)*workspace.dotweight(3));