%% plot_planes
%
% *Description:*  this is a slightly modified version ofthe plot planes from the the
% surface growing work, it gets a structure of planes passed to it and the
% size mew and then makes planes out of the points where the plane
% intersects with a sphere translated to the center of the plane

%% Function Call
% 
% *Inputs:* 
%
% _plane_ (struct) planes structure with equations and home points
%
% _mew_ (double) radius of the planes
%
% *Returns:* NULL

function plot_handles=plot_planes(plane,mew,plotfill,plotColor)

%% allow people to pass in wether to fill the disks or not
if nargin<4
    plotColor=[0,0,1];
    if nargin<3
        plotfill=true;
    end
end

%% New Variables
[x,y,z]=ellipsoid(0,0,0,mew,mew,0,50);
sqrd_dist=x.^2+y.^2+z.^2;
x=x(sqrd_dist>=mew^2);
y=y(sqrd_dist>=mew^2);
z=z(sqrd_dist>=mew^2);

%for each plane
plot_handles=zeros([length(plane),1]);
hold on;

%% Go throuh each plane and transform the points from the ellipse
for i=1:length(plane)
  
    %angle between 2 lines
    angofrot=acos(dot(plane(i).equ(1:3),[0,0,1]));
    axisofrot=cross(plane(i).equ(1:3),[0,0,1]);
    val=rot_vec([x,y,z],axisofrot,angofrot);

    toplot=[val(:,1)+plane(i).home_point(1),...
            val(:,2)+plane(i).home_point(2),...
            val(:,3)+plane(i).home_point(3)];

     if plotfill
         plot_handles(i)=fill3(toplot(:,1),toplot(:,2),toplot(:,3),plotColor);        
     else
         plot_handles(i)=plot3(toplot(:,1),toplot(:,2),toplot(:,3),'k');        
     end    
end
