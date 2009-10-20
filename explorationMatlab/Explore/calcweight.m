%% calcweight
%
% *Description:* This is the port of call for calculating the weight of the
% occupancy grid

function weightval=calcweight(occupancy_enumerater)
% KNOWN (0)
if occupancy_enumerater==0
    weightval = calknownweight();
% UNKNOWN (0.5)
elseif occupancy_enumerater==0.5
    weightval = calunknownweight();
% OBSTACLE (1)
elseif occupancy_enumerater==1
    weightval = calobstacleweight();
else
    error('occupancy_enumerater is incorrect');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% calknownweight
%
% *Description:* this function calculated the weighted amount of known info.
% it uses the GetImpLevInfo() function to break up known points into
% their areas


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
             size(level2_kn,1)*workspace.dotweight(2)+...
             size(level3_kn,1)*workspace.dotweight(3));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
%% calobstacleweight
%
% *Description:* this function calculated the weighted amount of obstacle info.
% it uses the GetImpLevInfo() function to break up known points into
% their areas


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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
%% calunknownweight
%
% *Description:* this function calculated the weighted amount of unknown info.
% it uses the GetImpLevInfo() function to break up unknown points into
% their areas

function unknownweight = calunknownweight()

%% Variables
global workspace

%% Total points in the environment
% $$workspace=P_{u3} + P_{k1} + P_{o3}$$

%% Get UNKNOWN point matrices P_u, in Lev 1->3
% $$\begin{array}{l}
% P_{u1}:=(x_j,y_j,z_j)_{j=1,...,m_{u1}} \\ 
% P_{u2}:=(x_j,y_j,z_j)_{j=1,...,m_{u2}} \\
% P_{u3}:=(x_j,y_j,z_j)_{j=1,...,m_{u3}} \\\\
% P_{u1}\in P_{u2}\in P_{u3}
% \end{array}$$  

%remember level 1 is inside level 2 and 3 and level 2 is inside level 3
[level1_un,level2_un,level3_un]=GetImpLevInfo(workspace.unknowncoords);


%% Get KNOWN point matrices P_k, in Lev 1->3
% $$\begin{array}{l} 
% P_{k1}:=(x_j,y_j,z_j)_{j=1,...,m_{k1}} \\
% P_{k2}:=(x_j,y_j,z_j)_{j=1,...,m_{k2}} \\
% P_{k3}:=(x_j,y_j,z_j)_{j=1,...,m_{k3}} \\\\ 
% P_{k1}\in P_{k2}\in P_{k3}
% \end{array}$$  

%remember level 1 is inside level 2 and 3 and level 2 is inside level 3
[level1_kn,level2_kn,level3_kn]=GetImpLevInfo(workspace.knowncoords);


%% Get OBSTACLE point matrices P_o, in Lev 1->3
% $$\begin{array}{l} 
% P_{o1}:=(x_j,y_j,z_j)_{j=1,...,m_{o1}} \\
% P_{o2}:=(x_j,y_j,z_j)_{j=1,...,m_{o2}} \\
% P_{o3}:=(x_j,y_j,z_j)_{j=1,...,m_{o3}} \\\\
% P_{o1}\in P_{o2}\in P_{o3}
% \end{array}$$ 

%remember level 1 is inside level 2 and 3 and level 2 is inside level 3
[level1_ob,level2_ob,level3_ob]=GetImpLevInfo(workspace.indexedobsticles);


%% Take off known & obstacle and Multiply by weighting
% $$\begin{array}{l}
% W_{known}= \\
% C_{dot1}\times (m_{u1}-m_{k1}-m_{o1}) + \\
% C_{dot2}\times (m_{u2}-m_{k2}-m_{o2}) + \\
% C_{dot3}\times (m_{u3}-m_{k3}-m_{o3})
% \end{array}$$

%this is the difference in weighted unknown points
unknownweight=(size(level1_un,1)-size(level1_kn,1)-size(level1_ob,1))*workspace.dotweight(1)+...
              (size(level2_un,1)-size(level2_kn,1)-size(level2_ob,1))*workspace.dotweight(2)+...
              (size(level3_un,1)-size(level3_kn,1)-size(level3_ob,1))*workspace.dotweight(3);

if unknownweight<0
    unknownweight=0;
end
          