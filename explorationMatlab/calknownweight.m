%% calunknownweight
%
% *Description:* this function calculated the weighted amount of known info.
% it uses the GetImpLevInfo() function to break up known points into
% their areas

%% Function Call
% 
% * *Inputs:* Null
% * *Returns:* Null

function knownweight = calknownweight()

%% Variables
global workspace

%% Get known point matrices P, in Lev 1->3
% $$\begin{array}{l}
% P_1:=(x_j,y_j,z_j)_{j=1,...,m_1}, P_2:=(x_j,y_j,z_j)_{j=1,...,m_2},
% P_3:=(x_j,y_j,z_j)_{j=1,...,m_3}\\\\
% P_1\in P_2\in P_3
% \end{array}$$

%remember level 1 is inside level 2 and 3 and level 2 is inside level 3
[level1_kn,level2_kn,level3_kn]=GetImpLevInfo(workspace.knowncoords);

%% Multiply by weighting
% $$W_{known}=C_{dot1}\times m_1 + C_{dot2}\times m_2 + C_{dot3}\times
% m_3$$

%this is the difference in weighted unknown points
knownweight=(size(level1_kn,1)*workspace.dotweight(1)+...
             size(level2_kn,2)*workspace.dotweight(2)+...
             size(level3_kn,3)*workspace.dotweight(3));
