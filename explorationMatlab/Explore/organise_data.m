%% organise_data
%
% *Description:*  This function gets the data back either from loading or the
% robot and fills in the known free and known full the same as the do scan
% function does. Most things need to exist for this function, including the
% global variables describing the current workspace, the sensor data collected, the
% robot and all the G_scan.PointData, G_scan.RangeData from the scan

%% Function Call
%
% *Inputs:* NULL
%
% *Returns:* NULL
function organise_data()

%% Variables
global workspace G_scan robot_maxreach
hCOM = getappdata(gcf, 'hCOM');

% warning off;
%% Check data validity
% Process Data from G_scan.PointData matrix to a 3*many form
 ice_cream_bounds=zeros([size(G_scan.PointData,1)*size(G_scan.PointData,2),3]);
 laser_pos_ice_cream_bounds=zeros([size(G_scan.PointData,1)*size(G_scan.PointData,2),3]);
for i=1:size(G_scan.PointData,1);
    ice_cream_bounds((i-1)*size(G_scan.PointData,2)+1 : i*size(G_scan.PointData,2) , :)=...
        [G_scan.PointData(i,:,1)',G_scan.PointData(i,:,2)',G_scan.PointData(i,:,3)'];    
    laser_pos_ice_cream_bounds((i-1)*size(G_scan.PointData,2)+1 : i*size(G_scan.PointData,2) , :)=...
        ones([size(G_scan.PointData,2),1])*[G_scan.PoseData(i,1,4),G_scan.PoseData(i,2,4),G_scan.PoseData(i,3,4)];    
end


% Eliminate points where the range data is == 0
if ~isempty(G_scan.RangeData)
    rangedata_singlemat=zeros([size(G_scan.RangeData,1)*size(G_scan.RangeData,2),1]);
    raywidth=(2*G_scan.theta)/size(G_scan.RangeData,2);
    averagefiltering = filter2(fspecial('average',20),G_scan.RangeData);   
    noreturnplaces=zeros([size(averagefiltering,2),3]);
    laser_pos_noreturnplaces=zeros([size(averagefiltering,2),3]);    
    countnoreturns=0;
    countnoreturns_pose=1;
    
    for i=1:size(G_scan.RangeData,1);
        rangedata_singlemat((i-1)*size(G_scan.RangeData,2)+1 : i*size(G_scan.RangeData,2))=G_scan.RangeData(i,:)';                  
   
        noreturnstemp=find(averagefiltering(i,:)<20);
        %go to each point which isn't returned and make it and x,y,z at max
        if size(noreturnstemp,1)>0
            angularofnoreturn=(size(G_scan.RangeData,2)/2-noreturnstemp)*raywidth;

            %go through every second ray on the pan since if they are one
            %off they are not good, if they are multiple then 1 is enough
            %even at 1.6 met
            for j=1:2:size(angularofnoreturn,2)
                rotatedpoint=rot_vec(G_scan.range*G_scan.PoseData(i,1:3,3),G_scan.PoseData(i,1:3,1),-angularofnoreturn(j));

                countnoreturns=countnoreturns+1;
                noreturnplaces(countnoreturns,:)=rotatedpoint+G_scan.PoseData(i,1:3,4);
            end
            laser_pos_noreturnplaces(countnoreturns_pose:countnoreturns,:)=ones([countnoreturns-countnoreturns_pose+1,1])*G_scan.PoseData(i,1:3,4);
            countnoreturns_pose=countnoreturns+1;
        end
    end

    %resize so we remove any unnessesary ones
    noreturnplaces=noreturnplaces(1:countnoreturns,:);
    laser_pos_noreturnplaces=laser_pos_noreturnplaces(1:countnoreturns,:);
    
    ice_cream_bounds=ice_cream_bounds(rangedata_singlemat>20,:);
    laser_pos_ice_cream_bounds=laser_pos_ice_cream_bounds(rangedata_singlemat>20,:);
   
    if size(ice_cream_bounds,1)==0
        error('There is no data with range greater than 20');
    end
 
else
    error('There is some problem with the laser, no data has been returned');
end


%% The points we want to trace too
[totraceto,whichones]=unique(round([ice_cream_bounds;noreturnplaces]/workspace.inc_size)*workspace.inc_size,'rows');
laser_pos=[laser_pos_ice_cream_bounds;laser_pos_noreturnplaces];
laser_pos=laser_pos(whichones,:);

%% Setup ray tracing variables
% Min and max cubes
space_min_and_max=[workspace.min/workspace.inc_size,workspace.max/workspace.inc_size];
% The cubes that rays pass through
markedcubes=[];
%% Get distance from 1 laser_pos to end points
% $$ \begin{array}{l}
% Ps=laser\_pos=(x_{j},y_{j},z_{j})_{j=1,2...m}\\
% Pe=ice\_cream\_bounds=(x_{j},y_{j},z_{j})_{j=1,2...m}\\
% \forall m, dist_m=\sqrt{(P_{ex}-P_{sx})^2+(P_{ey}-P_{sy})^2+(P_{ez}-P_{sz})^2}
% \end{array}$$
dist=sqrt((laser_pos(:,1)-totraceto(:,1)).^2+...
          (laser_pos(:,2)-totraceto(:,2)).^2+...
          (laser_pos(:,3)-totraceto(:,3)).^2);
% the valid distances (greater than 0)
valid_rows=find(dist);

%% Setup ray tracing discrete check points
% $$\forall m, tempstarter_m=\frac{Pe_{mx}-Ps_{mx}}{2dist_m/inc\_size}$$
tempstarter=(totraceto(:,1)-laser_pos(:,1))./(2*dist(:)/workspace.inc_size);

%% Go through each valid row (where dist>0) and ray trace
for i=valid_rows'
%---collum 1
    %check each one of the segements for zero distance and fill with that planes value for inbetweens  
    if abs(laser_pos(i,1)-totraceto(i,1))<tempstarter(i)
        tempCOL=laser_pos(i,1)*ones([round((2*dist(i)/workspace.inc_size))+1,1]);
    else
        tempCOL=[(laser_pos(i,1):tempstarter(i):totraceto(i,1))'];       
    end
    inbetweenpoint=[tempCOL,ones([length(tempCOL),2])];

%---collum 2
    if laser_pos(i,2)==totraceto(i,2) || size(inbetweenpoint,1)<=1
          inbetweenpoint(:,2)=inbetweenpoint(:,2)*laser_pos(i,2);
     else
        %since sometimes due to a rounding error dividing it will not be the same, so minus 0.5 off the size and we should get the correct
        %num of rows so concaternation can happen properly
        try 
            tempCOL(1:end)=(laser_pos(i,2):(totraceto(i,2)-laser_pos(i,2))/(size(inbetweenpoint,1)-1):totraceto(i,2))';
        catch %#ok<CTCH> 
            tempCOL(1:end)=(laser_pos(i,2):(totraceto(i,2)-laser_pos(i,2))/(size(inbetweenpoint,1)-0.5):totraceto(i,2))';
        end
        inbetweenpoint(:,2)=tempCOL;
    end
%---collum 3    
    if laser_pos(i,3)==totraceto(i,3) || size(inbetweenpoint,1)<=1
        inbetweenpoint(:,3)=inbetweenpoint(:,3)*laser_pos(i,3);
    else
        try 
            tempCOL(1:end)=(laser_pos(i,3):(totraceto(i,3)-laser_pos(i,3))/(size(inbetweenpoint,1)-1):totraceto(i,3))';       
        catch %#ok<CTCH>
            tempCOL(1:end)=(laser_pos(i,3):(totraceto(i,3)-laser_pos(i,3))/(size(inbetweenpoint,1)-0.5):totraceto(i,3))';       
        end
        inbetweenpoint(:,3)=tempCOL;
    end

%% Determine the cubes which the ray passed through within work (generous)
% $$ cubes\_checked=\left[\begin{array}{c} 
% \lfloor \frac{inbetweenpoint}{inc\_size} \rfloor \\
% \frac{inbetweenpoint}{inc\_size}
% \end{array}\right]$$
    cubes_checked=[int16(floor(inbetweenpoint/workspace.inc_size));int16(round(inbetweenpoint/workspace.inc_size))];
    % Concaternate the cubes check which are in workspace with current list    
    markedcubes=[markedcubes;int16(cubes_checked((cubes_checked(:,1)>=space_min_and_max(1) &...
                                                 cubes_checked(:,2)>=space_min_and_max(2) &...
                                                 cubes_checked(:,3)>=space_min_and_max(3) &...
                                                 cubes_checked(:,1)<=space_min_and_max(4) &...
                                                 cubes_checked(:,2)<=space_min_and_max(5) &...
                                                 cubes_checked(:,3)<=space_min_and_max(6)),:))];        
        
    %this balances out with the above adding rows to a matrix, since unique
    %is a slow function approx 49x slower than array resizing done above
    if rand>0.98
        markedcubes=unique(markedcubes,'rows'); 
    end         
end                          

%% final step of saving the unique cubes rays passed through and end points
if size(markedcubes)>0
    markedcubes=unique(double(markedcubes),'rows');
    %put back in realworld coords
    points=markedcubes.*workspace.inc_size;
    %make sure all are still inside workspace
    points=points((points(:,1)>=workspace.min(1) & points(:,2)>=workspace.min(2) & points(:,3)>=workspace.min(3) &...
                   points(:,1)<=workspace.max(1) & points(:,2)<=workspace.max(2) & points(:,3)<=workspace.max(3)),:);
  
    %make sure end points are within the workspace bounds
    ice_cream_bounds=ice_cream_bounds((ice_cream_bounds(:,1)>=workspace.min(1) & ice_cream_bounds(:,1)<=workspace.max(1) &...
                                       ice_cream_bounds(:,2)>=workspace.min(2) & ice_cream_bounds(:,2)<=workspace.max(2) &...
                                       ice_cream_bounds(:,3)>=workspace.min(3) & ice_cream_bounds(:,3)<=workspace.max(3)),:);

    %remove self scanning points (points that are within the joints force fields)
    ice_cream_bounds_NOSELF=remove_self_scanning(ice_cream_bounds);
    
%     indexedobsticles=unique(floor(ice_cream_bounds_NOSELF/workspace.inc_size)*workspace.inc_size,'rows');
    indexedobsticles=putInVoxels_gp(ice_cream_bounds_NOSELF,workspace.inc_size);
    %also get rid of any indexed points that are within the feilds
    indexedobsticles=remove_self_scanning(indexedobsticles);
    


%% new method to try and make the iner circle of movement cleaner using
    %stephens data
    try 
      %make allowances for the RTA proj code
      aabb = [workspace.impLev(1).x(1), workspace.impLev(1).y(1) workspace.impLev(1).z(1);...
              workspace.impLev(1).x(2), workspace.impLev(1).y(2) workspace.impLev(1).z(2)];
      %make allowances for the RTA proj code      
      hMesh = hCOM.Surface.SurfacesInsideBox(aabb(1,:), aabb(2,:));
      if size(hMesh.VertexData,1)>0
        indexedobsticles_levl1=putInVoxels_gp(hMesh.VertexData(GetImpLevInfo(hMesh.VertexData),:),workspace.inc_size);
      else
        indexedobsticles_levl1=[];
      end
    catch  %#ok<CTCH>
      keyboard
      error('In getting the mesh needed for surface making');
    end    
    [level1,level2,level3]=GetImpLevInfo(indexedobsticles);
    %get points which are inside entire workspace but are not in the robots
    %range (I will use the points from the mesh which are more acurate)
    workspace.indexedobsticles=putInVoxels_gp([workspace.indexedobsticles;...
                                               indexedobsticles(setdiff(level3,level1),:);...
                                               indexedobsticles_levl1],workspace.inc_size);
end

%% Remove points that are on a path we have taken
if size(robot_maxreach.pointcarvedout,1)>0
    workspace.indexedobsticles=setdiff(workspace.indexedobsticles,robot_maxreach.pointcarvedout,'rows');
end

%% Fill in the newest knowledge about points
%the newest knowledge is what is now known compared to what was known about freespace and obstacles before
workspace.newestknownledge=setdiff(points,[workspace.knowncoords;workspace.indexedobsticles],'rows');
workspace.knowncoords=unique([workspace.knowncoords;points],'rows');
%overall point is what is known that is not an obstacle
workspace.knowncoords=setdiff(unique([workspace.knowncoords;points],'rows'),workspace.indexedobsticles,'rows');

%% Do the 3D median filtering on the unknown space
%threeDMedianFilt();

%% Do surface making on obstacle points from this exploration view 
% try if size(ice_cream_bounds_NOSELF,1)>1
%         surface_making_simple(ice_cream_bounds_NOSELF,workspace.mew);
%     end
try if size(workspace.indexedobsticles,1)>1
        surface_making_simple(workspace.indexedobsticles,workspace.mew);
    end        
catch %#ok<CTCH>
    keyboard; 
end
global plane

%% Add to made surface variables
% Clear it and reset everytime
workspace.indexedobsticles_home_point=[];
workspace.indexedobsticles_equ=[];
for i=1:size(plane,2)
   workspace.indexedobsticles_home_point=[workspace.indexedobsticles_home_point;...
       floor(plane(i).home_point/workspace.inc_size)*workspace.inc_size];
   workspace.indexedobsticles_equ=[workspace.indexedobsticles_equ;plane(i).equ];   
end

%% Determine the special area 
try special_map_area();
catch %#ok<CTCH> 
    display('Some problem with special points');
    keyboard;
end    
