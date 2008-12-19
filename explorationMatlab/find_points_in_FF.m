%% find_points_in_FF
%
% *Description:*  pass into this function a group of 3D points and pass in
% the newQ to find out which points are inside the elipsoids

%% Function Call
% 
% *Inputs:* 
%
% _points_ (3*m double) holds the points in space
%
% _newQ_ (6*1 double) holds the joint angles in radians
%
% includefactor (double) this is how big the ellisoids indicates how many
% points included
%
% *Returns:* 
%
% insidepoints (3*m double) points in ellipsoids

function insidepoints = find_points_in_FF(points,newQ,includefactor)

%% Variables
global r densoobj

% if we haven't been passed the joints then 
if nargin<3
    error('Must pass the includefactor and newQ');
end

insidepoints=[];

n = r.n;
L = r.link;
t = r.base;

%% Go through each link of robot (1->n)
for i=1:size(newQ,2)
    t = t * L{i}(newQ(i));
    % Translate points to the elispe coordinate frame (IE leave the elispes
    % where they were to start off with and translate the world around them
    translated_points=[points(:,1)-t(1,4) points(:,2)-t(2,4) points(:,3)-t(3,4)];
    translated_points=(t(1:3,1:3)'*translated_points')';

%% For the first 2 links: Get points outside this ellipse
% $$ \begin{array}{c}
% points_{remaining}=\forall [X,Y,Z] :: \\\\
% \frac{(X-center_x)^2}{a^2}+\frac{(Y-center_y)^2}{b^2}+\frac{(Z-center_z)^2}{c^2}<includefactor
% \end{array}$$
      insidepoints=[insidepoints;points((((translated_points(:,1)-densoobj(i+1).ellipse.center(1)).^2)/densoobj(i+1).ellipse.params(1)^2+...
                           ((translated_points(:,2)-densoobj(i+1).ellipse.center(2)).^2)/densoobj(i+1).ellipse.params(2)^2+...
                           ((translated_points(:,3)-densoobj(i+1).ellipse.center(3)).^2)/densoobj(i+1).ellipse.params(3)^2<includefactor),:)];

end

insidepoints=unique(insidepoints,'rows');
