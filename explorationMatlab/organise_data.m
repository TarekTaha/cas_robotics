%% organise_data
%
% *Description:*  This function gets the data back either from loading or the
% robot and fills in the known free and known full the same as the do scan
% function does. Most things need to exist for this function, including the
% global variables describing the current workspace, the scan taken, the
% robot and all the PointData, RangeData from the scan

%% Function Call
%
% *Inputs:* NULL
%
% *Returns:* NULL
function organise_data()

%% Variables
global workspace scan PointData RangeData PoseData robot_maxreach robmap_h

%optimising the surface map
robmap_h.Optimise

% starttime=clock;
% Number of points we want in the cube for ray casting
% numpntsInCube=scan.numpntsInCube;

% warning off;
%% Check data validity
% Process Data from PointData scan matrix to a 3*many form
 ice_cream_bounds=zeros([size(PointData,1)*size(PointData,2),3]);
 laser_pos_ice_cream_bounds=zeros([size(PointData,1)*size(PointData,2),3]);
for i=1:size(PointData,1);
    ice_cream_bounds((i-1)*size(PointData,2)+1 : i*size(PointData,2) , :)=...
        [[PointData(i,:,1)]',[PointData(i,:,2)]',[PointData(i,:,3)]'];    
    laser_pos_ice_cream_bounds((i-1)*size(PointData,2)+1 : i*size(PointData,2) , :)=...
        ones([size(PointData,2),1])*[PoseData(i,1,4),PoseData(i,2,4),PoseData(i,3,4)];    
end


% Eliminate points where the range data is == 0
if ~isempty(RangeData)
    rangedata_singlemat=zeros([size(RangeData,1)*size(RangeData,2),1]);
    noreturnplaces=[];
    laser_pos_noreturnplaces=[];
    raywidth=(2*scan.theta)/size(RangeData,2);
    averagefiltering = filter2(fspecial('average',20),RangeData);   
    noreturnplaces=zeros([size(averagefiltering,2),3]);
    laser_pos_noreturnplaces=zeros([size(averagefiltering,2),3]);    
    countnoreturns=0;
    countnoreturns_pose=1;
    
    for i=1:size(RangeData,1);
        rangedata_singlemat((i-1)*size(RangeData,2)+1 : i*size(RangeData,2))=RangeData(i,:)';                  

%         noreturnstemp=find(RangeData(i,:)<20);        
        noreturnstemp=find(averagefiltering(i,:)<20);
        %go to each point which isn't returned and make it and x,y,z at max
        if size(noreturnstemp,1)>0
            angularofnoreturn=(size(RangeData,2)/2-noreturnstemp)*raywidth;

%             figure(3);grid on; hold on; view(3);axis equal;

            %go through every second ray on the pan since if they are one
            %off they are not good, if they are multiple then 1 is enough
            %even at 1.6 met
            for j=1:2:size(angularofnoreturn,2)
%                 hgtr=makehgtform('axisrotate',[PoseData(i,1:3,1)],angularofnoreturn(j));                 
%                 rotatedpoint=scan.size*PoseData(i,1:3,3)*hgtr(1:3,1:3);
                rotatedpoint=rot_vec(scan.size*PoseData(i,1:3,3),PoseData(i,1:3,1),-angularofnoreturn(j));
                
%                 if ~isempty(find((rotatedpoint-rotatedpoint2)>eps, 1))
%                     keyboard
%                 end
                
%                 noreturnplaces=[noreturnplaces;rotatedpoint+PoseData(i,1:3,4)];
%                 laser_pos_noreturnplaces=[laser_pos_noreturnplaces;PoseData(i,1:3,4)];
                countnoreturns=countnoreturns+1;
                noreturnplaces(countnoreturns,:)=rotatedpoint+PoseData(i,1:3,4);
%                 laser_pos_noreturnplaces(countnoreturns,:)=PoseData(i,1:3,4);
                
                
%                 plot3(noreturnplaces(end,1),noreturnplaces(end,2),noreturnplaces(end,3),'r.')
%                 pause(0.1);
            end
            laser_pos_noreturnplaces(countnoreturns_pose:countnoreturns,:)=ones([countnoreturns-countnoreturns_pose+1,1])*PoseData(i,1:3,4);
            countnoreturns_pose=countnoreturns+1;
        end
    end

    %resize so we remove any unnessesary ones
    noreturnplaces=noreturnplaces(1:countnoreturns,:);
    laser_pos_noreturnplaces=laser_pos_noreturnplaces(1:countnoreturns,:);
    
    ice_cream_bounds=ice_cream_bounds(rangedata_singlemat>20,:);
    laser_pos_ice_cream_bounds=laser_pos_ice_cream_bounds(rangedata_singlemat>20,:);
    
%     figure(3);plot3(noreturnplaces(:,1),noreturnplaces(:,2),noreturnplaces(:,3),'r.')
%     hold on; plot3(ice_cream_bounds(:,1),ice_cream_bounds(:,2),ice_cream_bounds(:,3),'b.')


    
    if size(ice_cream_bounds,1)==0
        error('There is no data with range greater than 20');
    end
    % Save this scan data as a block
%     workspace.ALLlastscandataInWkspace=ice_cream_bounds;    
else
    error('There is some problem with the laser, no data has been returned');
end


%% The points we want to trace too
[totraceto,whichones]=unique(round([ice_cream_bounds;noreturnplaces]/workspace.inc_size)*workspace.inc_size,'rows');
laser_pos=[laser_pos_ice_cream_bounds;laser_pos_noreturnplaces];
laser_pos=laser_pos(whichones,:);

%% Setup ray tracing variables
% Where the laser is at start of scan (used thoughout as origin)
% laser_pos=scan.origin;
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
for i=[valid_rows]'
%---collum 1
    %check each one of the segements for zero distance and fill with that planes value for inbetweens  
    if abs(laser_pos(i,1)-totraceto(i,1))<tempstarter(i)
        tempCOL=[laser_pos(i,1)*ones([round((2*dist(i)/workspace.inc_size))+1,1])];
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
        catch; 
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
        catch; 
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
      global robmap_h
      aabb = [workspace.impLev(1).x(1), workspace.impLev(1).y(1) workspace.impLev(1).z(1);...
              workspace.impLev(1).x(2), workspace.impLev(1).y(2) workspace.impLev(1).z(2)];
      hMesh = robmap_h.Mesh(aabb);
      if size(hMesh.VertexData,1)>0
        indexedobsticles_levl1=putInVoxels_gp(hMesh.VertexData(GetImpLevInfo(hMesh.VertexData),:),workspace.inc_size);
      else
        indexedobsticles_levl1=[];
      end
    catch; 
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

%% Timing and Display purposes
% temptime=etime(clock,starttime);
% display (strcat('You filled in:',num2str(size(points)),' cubes in: ',num2str(temptime),'secs'));

%% Update (indexed) obstacles points global variables
    % workspace.Nobsticlepoints=[workspace.Nobsticlepoints;ice_cream_bounds_NOSELF];
    % only want unique indexed obsticles
%     if size(workspace.indexedobsticles,2)==0
%         workspace.indexedobsticles=indexedobsticles;
%     else
%         workspace.indexedobsticles=union(indexedobsticles,workspace.indexedobsticles,'rows');
%     end

%% Remove points that are on a path we have taken
if size(robot_maxreach.pointcarvedout,1)>0
    workspace.indexedobsticles=setdiff(workspace.indexedobsticles,robot_maxreach.pointcarvedout,'rows');
end

%% Fill in the newest knowledge about points
%the newestscan knowledge is what is now known compared to what was known about freespace and obstacles before
workspace.newestknownledge=setdiff(points,[workspace.knowncoords;workspace.indexedobsticles],'rows');
workspace.knowncoords=unique([workspace.knowncoords;points],'rows');
%overall point is what is known that is not an obstacle
workspace.knowncoords=setdiff(unique([workspace.knowncoords;points],'rows'),workspace.indexedobsticles,'rows');

%% Do the 3D median filtering on the unknown space
%threeDMedianFilt();

%% Do surface making on obstacle points from this scan 
% try if size(ice_cream_bounds_NOSELF,1)>1
%         surface_making_simple(ice_cream_bounds_NOSELF,workspace.mew);
%     end
try if size(workspace.indexedobsticles,1)>1
        surface_making_simple(workspace.indexedobsticles,workspace.mew);
    end        
catch; keyboard; end
global plane

%% Add to made surface variables
%this makes a workspace indexed version of the home points
% for i=1:size(plane,2)
%    workspace.indexedobsticles_home_point=[workspace.indexedobsticles_home_point;...
%        floor(plane(i).home_point/workspace.inc_size)*workspace.inc_size];
%    workspace.indexedobsticles_equ=[workspace.indexedobsticles_equ;plane(i).equ];   
% end

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
catch; display('Some problem with special points');
    keyboard;
end    
% warning on