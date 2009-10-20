%% FUNCTION: basic_scan
%
% Description: this traces out the lines and gets a group of points as the
% basic scan 

%% Function Call
%
% *Inputs:* 
% 
% _file_name_ (size, type string) what is its purpose?
%
% *Returns:* 
% 
% _points_ (3 *many) points where the laser rays finishes (discretised)
%
% _ice_cream_bounds_ (3 *many) points but undiscretised.... I think
% 

function [points,ice_cream_bounds]=basic_scan(cube_size)

%% Variables
global G_scan
%starttime=clock;

%this will actually be x or y axis from end effector but not sure which
%have to find out which axis the 5th joint rotates around
laser_pos=[0,0,0];
bear=[1,0,1]; %used to be [0,0,1]
tilt_rotate_vec=[0,1,0];

%max range of laser
las_range=G_scan.size;

%Laser Angualar VARIABLES
%this is the angle either side of the bearing of the center of the sensor ray \|/
theta=G_scan.theta;
%this is the angle from the tilt, - is up, + is down, angle must be from -2pi to 2pi
alpha=-G_scan.alpha;


% %SETUP WORKSPACE

%since we want to know what is the max angle we can use HERE only so that
%at the furthest point we have resolution enough to cover our cubes at
%least 3 times at the point of longest laser range, so we use that angular
%resolution for this simulation
theta_incr=atan((cube_size/3)/las_range);
%we have an actual min resoltion in hardware so this is the minmum
if theta_incr<0.36*(pi/180);
    theta_incr=0.36*(pi/180);
end
    
%% Take the scan - work out end points
% This is the most important vector
% it describes the center of the first laser pan ray 
% we will rotate to ge the pan scan
dir_vec=las_range*(bear);
%this is the vector that we will pan rotate around, it is always at origin
%so we get cross product of the two vectors on the plane to get normal
%pan_rotate_vec=cross(dir_vec,tilt_rotate_vec);
pan_rotate_vec=tilt_rotate_vec  * [0 -dir_vec(3) dir_vec(2);...
                                   dir_vec(3) 0 -dir_vec(1);...
                                   -dir_vec(2) dir_vec(1) 0];


%this works out through pan rotation of the ice cream cone from the origin
%work out how many increments based on lasers lowest steps
increments=round(2*theta/theta_incr);
single_pan=[]; 
for j=-theta:(2*theta)/(increments-1):theta
    single_pan=[single_pan;rot_vec(dir_vec,pan_rotate_vec,j)];
end
current_row=1;
ice_cream_bounds=zeros([increments^2,3]);

%this tilt rotates and transforms to come form laser position
for i=0:alpha/(increments-1):alpha
    tilt_rot_res=rot_vec(single_pan,tilt_rotate_vec,i);
    new_current_row=current_row+size(tilt_rot_res,1);
    ice_cream_bounds(current_row:new_current_row-1,:)=[tilt_rot_res(:,1)+laser_pos(1),...
        tilt_rot_res(:,2)+laser_pos(2),...
        tilt_rot_res(:,3)+laser_pos(3)];
    current_row=new_current_row;    
end

%% Find cubes which are intersected with and fill them with a point
markedcubes=[];

dist=sqrt((laser_pos(1)-ice_cream_bounds(:,1)).^2+...
          (laser_pos(2)-ice_cream_bounds(:,2)).^2+...
          (laser_pos(3)-ice_cream_bounds(:,3)).^2);
valid_rows=find(dist);

tempstarter=(ice_cream_bounds(:,1)-laser_pos(1))./(2*dist(:)/cube_size);


for i=[valid_rows]'
    
    %check each one of the segements for zero distance and fill with that planes value for inbetweens  
    if laser_pos(1)==ice_cream_bounds(i,1)
        inbetweenpoint=[laser_pos(1)*ones([round((2*dist(i)/cube_size))+1,1])];
    else
        %inbetweenpoint=[(laser_pos(1):(ice_cream_bounds(i,1)-laser_pos(1))/(2*dist(i)/cube_size):ice_cream_bounds(i,1))'];
        inbetweenpoint=[(laser_pos(1):tempstarter(i):ice_cream_bounds(i,1))'];

    end
    if laser_pos(2)==ice_cream_bounds(i,2)
        inbetweenpoint=[inbetweenpoint,(laser_pos(2)*ones([size(inbetweenpoint,1),1]))];
    else
        inbetweenpoint=[inbetweenpoint,(laser_pos(2):(ice_cream_bounds(i,2)-laser_pos(2))/(size(inbetweenpoint,1)-1):ice_cream_bounds(i,2))'];
    end
    if laser_pos(3)==ice_cream_bounds(i,3)
        inbetweenpoint=[inbetweenpoint,(laser_pos(3)*ones([size(inbetweenpoint,1),1]))];
    else
        inbetweenpoint=[inbetweenpoint,(laser_pos(3):(ice_cream_bounds(i,3)-laser_pos(3))/(size(inbetweenpoint,1)-1):ice_cream_bounds(i,3))'];
    end   
  %end 
  %workspace_origin
  cubes_checked=floor(inbetweenpoint/cube_size);

    %Note: since this may be rotated we don't want to delete potential
    %infomation (deleted the statement which did this

    %had to add this
    markedcubes=[markedcubes;cubes_checked];      

    %this balances out with the above adding rows to a matrix
    if rand>0.98
        markedcubes=unique(markedcubes,'rows'); 
    end
%  end           
end

% final step of saving the unique points
if size(markedcubes)>0
    markedcubes=unique(markedcubes,'rows');
    points=markedcubes.*cube_size;
end

%etime(clock,starttime)

% %% Plot the ice cream on the pyrimid cone
% %This works out the bounds of the pyramid (only needed for plotting)
% pan_pos_max=rot_vec(dir_vec,pan_rotate_vec,theta)+laser_pos;
% pan_neg_max=rot_vec(dir_vec,pan_rotate_vec,-theta)+laser_pos;
% tilt_pos_max=rot_vec(pan_pos_max-laser_pos,tilt_rotate_vec,alpha)+laser_pos;
% tilt_neg_max=rot_vec(pan_neg_max-laser_pos,tilt_rotate_vec,alpha)+laser_pos;
% 
% hold on;axis equal;
% scan_line_handles=[];
% for k=1:round(increments/10):increments
%     %col
%     col_lines=(k-1)*increments+(1:increments);       
%     scan_line_handles=[scan_line_handles;...
%         plot3(ice_cream_bounds(col_lines,1),ice_cream_bounds(col_lines,2),ice_cream_bounds(col_lines,3));...
%         plot3([ice_cream_bounds(col_lines(1),1),laser_pos(1),ice_cream_bounds(col_lines(end),1)],...
%               [ice_cream_bounds(col_lines(1),2),laser_pos(2),ice_cream_bounds(col_lines(end),2)],...
%               [ice_cream_bounds(col_lines(1),3),laser_pos(3),ice_cream_bounds(col_lines(end),3)])];
%     
%      %hor
%     row_lines=k:increments:size(ice_cream_bounds,1);    
%     scan_line_handles=[scan_line_handles;...
%         plot3(ice_cream_bounds(row_lines,1),ice_cream_bounds(row_lines,2),ice_cream_bounds(row_lines,3));...    
%         plot3([ice_cream_bounds(row_lines(1),1),laser_pos(1),ice_cream_bounds(row_lines(end),1)],...
%               [ice_cream_bounds(row_lines(1),2),laser_pos(2),ice_cream_bounds(row_lines(end),2)],...
%               [ice_cream_bounds(row_lines(1),3),laser_pos(3),ice_cream_bounds(row_lines(end),3)])];
% 
% end
% 
% scan_point_handles=[];
% scan_point_handles=[scan_point_handles;...
%     plot3(laser_pos(1),laser_pos(2),laser_pos(3),'r*');...
%     plot3(dir_vec(1)+laser_pos(1),dir_vec(2)+laser_pos(2),dir_vec(3)+laser_pos(3),'y*');...
%     ...
%     plot3(pan_pos_max(1),pan_pos_max(2),pan_pos_max(3),'b*');...
%     plot3(pan_neg_max(1),pan_neg_max(2),pan_neg_max(3),'b*');...
%     plot3([laser_pos(1),pan_pos_max(1)],[laser_pos(2),pan_pos_max(2)],[laser_pos(3),pan_pos_max(3)]);...
%     plot3([laser_pos(1),pan_neg_max(1)],[laser_pos(2),pan_neg_max(2)],[laser_pos(3),pan_neg_max(3)]);...
%     ...
%     plot3(tilt_pos_max(1),tilt_pos_max(2),tilt_pos_max(3),'g*');...
%     plot3(tilt_neg_max(1),tilt_neg_max(2),tilt_neg_max(3),'g*');...
%     plot3([laser_pos(1),tilt_pos_max(1)],[laser_pos(2),tilt_pos_max(2)],[laser_pos(3),tilt_pos_max(3)]);...
%     plot3([laser_pos(1),tilt_neg_max(1)],[laser_pos(2),tilt_neg_max(2)],[laser_pos(3),tilt_neg_max(3)])];
% 
% xlabel('x - Pan around');ylabel('y - Tilt Around');zlabel('z - stream point');grid on;hold on;
% 
% 
% %mark the point of the cube as the closest point in cube to workspace origin 
% if size(markedcubes)>0 
% figure(2)
% plot3(markedcubes(:,1)*cube_size,markedcubes(:,2)*cube_size,markedcubes(:,3)*cube_size,'r.');
% keyboard
% end  
% 
% % Delete the scanning handles
% delete_scan_lines(scan_line_handles,scan_point_handles)
% clear scan_line_handles scan_point_handles


%% FUNCTION: delete_scan_lines
% This cleans up the lines from a scan

function delete_scan_lines(scan_line_handles,scan_point_handles)

for i=1:size(scan_point_handles,1)
    delete(scan_point_handles(i));
end
for i=1:size(scan_line_handles,1)
    delete(scan_line_handles(i));
end
