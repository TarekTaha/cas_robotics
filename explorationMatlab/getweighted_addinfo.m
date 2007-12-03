%% getweighted_addinfo
%
% *Description:*  this function will consider the different areas of interest
% that have been concluded to be addinfo and will end up at a weighted
% value

%% Function Call
%
% *Inputs:* 
%
% _addinfo_ (3*many double) group of point we want to know weight of
%
% *Returns:* 
%
% _weighted_addinfo_ (double) weighted info value

function weighted_addinfo=getweighted_addinfo(addinfo)

%% Variables
global workspace
weighted_addinfo=0;

%% If points passed
if ~isempty(addinfo)
    %Get level info    
    [level1,level2,level3]=GetImpLevInfo(addinfo);
%% Multiply by the dot weights    
% $$\begin{array}{c}
% P_{level1}:=(x_j,y_j,z_j)_{j=1,...,m_{u1}} \\ 
% P_{level2}:=(x_j,y_j,z_j)_{j=1,...,m_{u2}} \\
% P_{level3}:=(x_j,y_j,z_j)_{j=1,...,m_{u3}} \\\\
% weighted\_addinfo= C_{dotweight1} \times m_{u1} +
%                    C_{dotweight2} \times m_{u2} +
%                    C_{dotweight2} \times m_{u3}
% \end{array}$$  

    weighted_addinfo=length(level1)*workspace.dotweight(1)+...
                     length(level2)*workspace.dotweight(2)+...
                     length(level3)*workspace.dotweight(3);
    
%% add on the weight of the special points
     if size(workspace.spec_pnts,1)>0
%          if size(intersect(addinfo(level2,:),workspace.spec_pnts,'rows'),1)>0
%              display([num2str(size(intersect(addinfo,workspace.spec_pnts,'rows'),1))]);
%              temp=intersect(addinfo(level2,:),workspace.spec_pnts,'rows')
% 
%              temph=plot3(temp(:,1),temp(:,2),temp(:,3),'r.');
%              keyboard             
%              try delete(temph);end
%          end

         weighted_addinfo=weighted_addinfo+...
             size(intersect(addinfo(level2,:),workspace.spec_pnts,'rows'),1)*workspace.dotweight(1);         
     end
end