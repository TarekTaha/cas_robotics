%% check_FF
%
% *Description:*  This function takes a transform and an ellipse definition
% and translates the points back to the original cordinate frame of the
% ellipse. Note the reason for this is because the ellipse has been
% translated because of the movement of the link of the robot which it
% encompases. This function simply returns true/false as to whether there
% are any points inside the ellipse in question. 
% _Note_ eventually I want to put in unknown points and an acceptable limit 

%% Function Call
% 
% *Inputs:* 
%
% _t_ (4*4 double) this is the transform of the arm,
%
% _ellipse_vals_ (struct) holds the center and parameters of the ellipse
%
% _points_ (3*m double) holds the points in space
%
% *Returns:* 
%
% _result_ (bin) =1 if ok, =0 if NOT ok

function result = check_FF(t,ellipse_vals,points)

%% Variables
if nargin<3
    %import point from somewhere
    global workspace
    % Avoiding only indexed obstacle points
    points=workspace.indexedobsticles;
    
    % Suggested way to include unknown points since unknowncoords-knowncoords inludes obstacles 
%     points=setdiff(workspace.unknowncoords(GetImpLevInfo(workspace.unknowncoords),:),...
%                    workspace.knowncoords(GetImpLevInfo(workspace.knowncoords),:),'rows');    
end
    
%assumed there's nothing to collide with
result=true;

%% As long as there are some points
% $$P \neq \emptyset $$
numpoints=size(points,1);
if numpoints>0

%% Translate Points back to coordinate frame of ellipse
% $$ P_T = P \times Trans(-p_x,-p_y,-p_z) \times 
% \begin{array}{ccc} 
% n_x & o_x & a_x \\ 
% n_y & o_y & a_y \\ 
% n_z & o_z & a_z\end{array}$$

    %we want to inverse translate the points IE leave the elispes where
    %they were to start off with and translate the world around them
%slow way
%    translated_points=[points(:,1)-t(1,4) points(:,2)-t(2,4) points(:,3)-t(3,4)];
%faster way
    translated_points_1=points(:,1)-t(1,4);
    translated_points_2=points(:,2)-t(2,4);
    translated_points_3=points(:,3)-t(3,4);       
    

%% Check if there are points in ellipse
% $$ \begin{array}{ccc}
% \forall p_T \in P_T: &  
% \frac{(p_{Tx}-center_x)^2}{a^2}+
% \frac{(p_{Ty}-center_y)^2}{b^2}+
% \frac{(p_{Tz}-center_z)^2}{c^2}<=1 & \Rightarrow
% result = 0 \end{array}$$

%old slow way
%     if ~isempty(find(((translated_points(:,1)-ellipse_vals.center(1)).^2)/ellipse_vals.params(1)^2+...
%                      ((translated_points(:,2)-ellipse_vals.center(2)).^2)/ellipse_vals.params(2)^2+...
%                      ((translated_points(:,3)-ellipse_vals.center(3)).^2)/ellipse_vals.params(3)^2<=1,1))
%         result=0; % THERE IS some point is inside
%     end  

% %hyperfast (version 4) method    
    translated_points_1_t=translated_points_1*t(1,1)+translated_points_2*t(2,1)+translated_points_3*t(3,1);
    index_1=find(abs(translated_points_1_t-ellipse_vals.center(1))<=abs(ellipse_vals.params(1)));    
    if ~isempty(index_1)
        translated_points_2_t=translated_points_1*t(1,2)+translated_points_2*t(2,2)+translated_points_3*t(3,2);        
        index_2=find(abs(translated_points_2_t(index_1)-ellipse_vals.center(2))<=abs(ellipse_vals.params(2)));    
        if ~isempty(index_2)
            translated_points_3_t=translated_points_1*t(1,3)+translated_points_2*t(2,3)+translated_points_3*t(3,3);
            index_3=find(abs(translated_points_3_t(index_1(index_2))-ellipse_vals.center(3))<=abs(ellipse_vals.params(3)));    
            if ~isempty(index_3)
                if ~isempty(find(((translated_points_1_t(index_1(index_2(index_3)))-ellipse_vals.center(1)).^2)/ellipse_vals.params(1)^2+...
                                 ((translated_points_2_t(index_1(index_2(index_3)))-ellipse_vals.center(2)).^2)/ellipse_vals.params(2)^2+...
                                 ((translated_points_3_t(index_1(index_2(index_3)))-ellipse_vals.center(3)).^2)/ellipse_vals.params(3)^2<=1,1))
                    result=false; % THERE IS some point is inside
                end
            end
        end
    end

end