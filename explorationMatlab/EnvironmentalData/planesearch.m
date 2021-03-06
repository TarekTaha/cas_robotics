%% planesearch
%
% *Description*: This function searches through plane data to make up
% bigger surfaces
 function planesearch (dosurfacemaking)
close all;

if nargin==0
    dosurfacemaking=true;
end
global workspace r Q robot_maxreach hMesh;

display('Running plane search, currently this is standalone, only for testing and for AFTER mapping');

%% Load dataset 
% Recomended that you use this on a set of points
%
%Do you want to load planes or make planes
% =2 if done externally
loadplanes1_makeplanes0=1;

if loadplanes1_makeplanes0
% Here are 3 examples (only run one at a time obviously)
% _1_
%load RoofPlaneSet.mat
% _2_ the point cloud data is a bit crappy
% load example_plane.mat; planeSet=plane;
% _3_
 load meshNplanes.mat; planeSet=plane;

elseif loadplanes1_makeplanes0==0
    %OR could use stephens planes
    display('Loading map');
    try 
        hCOM=getappdata(gcf,'hCOM');
        if isempty(hCOM.Surface) 
            error('There is no map handle');
        end
        %get the Denso Blasting Cost Function
        try global DensoBlasting_h
            if isempty(DensoBlasting_h);DensoBlasting_h = hCOM.Surface.GetDensoBlastingCostFunction; end
        catch
            display('EyeInHand Problem: Unable to create DensoBlastingCost from surface map')
        end
    catch
        display('EyeInHand Problem: Unable to create surface map')
    end
    directory=pwd;
    hCOM.Surface.Resolution=0.01;
    hCOM.Surface.SurfaceFusionDistance = 0.1;
    hCOM.Surface.SurfaceDeviation = 0.01;
    aabbExtent = [-2, -1, 0; 2, 1, 3];
    hCOM.Surface.Extent = aabbExtent;
    hCOM.Surface.AddRangeGrid([directory,'\','grid_1.ply']);
    aabb.lower = [-2, -1, 0];
    aabb.upper = [2, 1, 3];
    if isempty(hMesh)
        hMesh = hCOM.Surface.SurfacesInsideBox(aabb.lower, aabb.upper);
    end
    % f = hMesh.FaceData;
    v = hMesh.VertexData;
    %get verts in workspace
    [level1,level2,level3]=GetImpLevInfo(v);
    display('Doing surface making');
    surface_making_simple(v(level3,:),0.04);
    global plane
    planeSet=plane;
    
%% This is if it is done externally
elseif loadplanes1_makeplanes0==2
    global plane
    planeSet=plane;

end

%% Variables
maxDistConstant=0.2; %meters
maxAngleConstant=5*pi/180; %degrees
maxDist2PlaneConstant=0.05; %meters
% first 3 angles are based upon optimise.max_angle_for123 (force field safety)
% maxQConstant=[8*pi/180,10*pi/180,14*pi/180,20*pi/180,20*pi/180,20*pi/180];
maxQConstant=[20*pi/180,20*pi/180,20*pi/180,22*pi/180,22*pi/180,40*pi/180];
minplanes2callAsurface=4; % What is the minimum planes which make a surface worth growing
doposesel=true;
showClusterNormalDist=false; % do you want to see the angle between all normals in surface population
plot_all_poseInfo=false;
poseselect_messagesON=true;
saveposedata=false;
draw_center_of_all_tiles=false;
plotconnectivityGraphs=false;



%% Other variables set for all centers and normals
all_centers=zeros([size(planeSet,2),3]);
all_norms=zeros([size(planeSet,2),3]);

% stack variables with plane data
for i=1:size(planeSet,2)
    all_centers(i,:)=planeSet(i).home_point;
    all_norms(i,:)=planeSet(i).normal_by_eigenval';
end


% Make a graph with all nodes
connectivityGraph=zeros([size(planeSet,2),size(planeSet,2)]);


% Registered to surface
registered_to_surface=zeros([size(planeSet,2),1]);

%% If you want pose sel connectivity do this now
if doposesel
    if isempty(r)||isempty(Q); error('You must run exGUI once if you wish to do pose selection');end
    
%     setuprobot(7)
    
    %make the third
    display('moving to a better position')        
    
    NOhandleOPTIONS.useRealRobot=false;NOhandleOPTIONS.show_robot=false;NOhandleOPTIONS.animate_move=false;NOhandleOPTIONS.remv_unkn_in_mv=false;
    if ~isempty(find(Q-robot_maxreach.default_Q(end,:)>eps,1))
        try movetonewQ(0,robot_maxreach.default_Q(end,:),[],NOhandleOPTIONS)
        catch
            display('cant use moveto command, simply setting Q to desired pose');
            Q=robot_maxreach.default_Q(end,:);
        end
    end
    
    %make sure robot is a 7 link object for blasting
    

    %select planes within range
    [level1,level2,level3]=GetImpLevInfo(all_centers);
    
    % setup variables
    all_centers_new=[];
    all_norms_new=[];
    planeSet_new=[];
    % Go through targets within blasting range of this robot model
    for i=level2'
        if isempty(planeSet_new)
            all_centers_new=all_centers(i,:);
            all_norms_new=all_norms(i,:);
            planeSet_new=planeSet(i);
        else
            all_centers_new=[all_centers_new;all_centers(i,:)];
            all_norms_new=[all_norms_new;all_norms(i,:)];
            planeSet_new(end+1)=planeSet(i);
        end
    end
    all_centers=all_centers_new;
    all_norms=all_norms_new;
    planeSet=planeSet_new;
    
    % Update graph with all nodes
    connectivityGraph=zeros([size(planeSet,2),size(planeSet,2)]);

    % Update Registered to surface variable 
    registered_to_surface=zeros([size(planeSet,2),1]);

    % Plot the planes and the robot 
%     figure;plot(r,Q);hold on;plot3(all_centers(:,1),all_centers(:,2),all_centers(:,3),'r.');light

     tic
    display('Adding obstacles for pose selection');
    workspace.indexedobsticles=putinVoxels_gp([workspace.indexedobsticles;all_centers],workspace.inc_size);

%     profile clear;profile on;
      temp_poses=PoseSel4planesearch(planeSet,poseselect_messagesON);
      
      error('This may not work anymore - use tests from Ch5 of thesis')

      
% load meshNplanes_poseset2.mat
%load roofPlaneSet_poseset.mat
%     profile off;profile viewer;

    % Go throgh each of the poses returned and put into a variable all_poses
    % which also includes if the pose is valid or not as the 8th row of the matrix 
    all_poses=zeros([size(temp_poses,2),size(temp_poses(1).Q,2)+1]);
    for i=1:size(temp_poses,2)
        all_poses(i,:)=[temp_poses(i).Q,temp_poses(i).validPose];
    end
    
    % If we want to plot the info about the variance and mean and rose
    % histogram of the poses found
    if plot_all_poseInfo
        figure;
        for i=1:5%size(temp_poses(i).Q,2)
            subplot(2,3,i)        

            rose(all_poses(:,i),80)
            title(['Joint ',num2str(i), ',u=',num2str(mean(all_poses(:,i)*180/pi)), ' sig^2=',num2str((std(all_poses(:,i)*180/pi)^2)) ])
        end
        subplot(2,3,6)          
        warning off;
        pie([length(find(all_poses(:,8)==true)),length(find(all_poses(:,8)==false))]);
        warning on;
%         keyboard
    end
	toc
    if saveposedata
        save(['posedata_',num2str(datestr(now,'yyyymmddTHHMMSS')),'.mat'],'temp_poses');
    end
end


tic

%% DELETE ME
connectivityGraph_C1=connectivityGraph;
connectivityGraph_C2=connectivityGraph;
connectivityGraph_C3=connectivityGraph;
connectivityGraph_C4=connectivityGraph;



%% Search at each plane for surrounding planes 
% $$ \begin{array}{c} 
% D=|P_{i}-P| \\
% \theta=\cos^{-1}(n_{i} \dot P) \\
% \theta(\theta>\frac{\pi}{2})=\pi-\theta(\theta>\frac{\pi}){2})\\
% dis_{to\_plane}=\left|\frac{AP_x + BP_y + CP_z + D}{\sqrt{A^2+B^2+C^2}}\right|\\
% \end{array}$$
%1) within a distance
%2) where the angle between normals is small
for i=1:size(planeSet,2) 

% Optimisation Note 1: You dont need to sqrt the distance just make the later
% constant a squared value by nature; 2: Note how we only gather data and
% search i->end so there is no search overlap
    dist_to_all_TEMP=sqrt((all_centers(i,1)-all_centers(i:end,1)).^2+...
                          (all_centers(i,2)-all_centers(i:end,2)).^2+...
                          (all_centers(i,3)-all_centers(i:end,3)).^2);

% Optimisation Note 1: to get the angle you dont really need to normalise the vectors since
%they are already normalised; 2: Make sure comparitor is in radians to save a
% conversion here to degrees; 3: Note how we are only searching from i->end
% this is so we are not searching over the whole graph everytime since
% search would have much more overlap
    norm_ang_to_all_TEMP=acos(dot([all_norms(i,1)*ones(size(all_norms,1)-i+1,1),...
                                   all_norms(i,2)*ones(size(all_norms,1)-i+1,1),...
                                   all_norms(i,3)*ones(size(all_norms,1)-i+1,1)],all_norms(i:end,:),2));
    % Make sure all values are between 0 and 90 (change great values so they are 
    % eg 176'=4' and 130'=50'
    norm_ang_to_all_TEMP(norm_ang_to_all_TEMP>pi/2)=pi-norm_ang_to_all_TEMP(norm_ang_to_all_TEMP>pi/2);
    
    %make a temporary plane equation out of the ith normal and center point
    temp_plane_equ=[all_norms(i,1),all_norms(i,2),all_norms(i,3),...
                          -(all_norms(i,1)*all_centers(i,1)+all_norms(i,2)*all_centers(i,2)+all_norms(i,3)*all_centers(i,3))];
    %this is the distance from the ith plane to all other points
    dist_plane_to_points=dis_bet_plane_n_pnt_internal(temp_plane_equ,all_centers(i:end,:));
    
    %This is the Q difference between 1 pose and all other
    anglebetweenQs=[abs(all_poses(i,1)-all_poses(i:end,1)),...
                    abs(all_poses(i,2)-all_poses(i:end,2)),...
                    abs(all_poses(i,3)-all_poses(i:end,3)),...
                    abs(all_poses(i,4)-all_poses(i:end,4)),...
                    abs(all_poses(i,5)-all_poses(i:end,5)),...
                    abs(all_poses(i,6)-all_poses(i:end,6))];
    % the distance from a pose to another which is invalid is infinite
	anglebetweenQs(all_poses(i:end,8)==0,:)=inf;
    
%% Forms the connectivity graph
% $$ \begin{array}{c} 
% \mbox{Find i corresponding to connected planes}\\
% D_i<\mbox{maxDistConstant}  \\
% \theta_i<\mbox{maxAngleConstant}\\
% dis_{to\_plane,i}<\mbox{maxDist2PlaneConstant}\\
% \end{array}$$

    %Zeros stuffed at start to make updating the connectivity graph easier     
    if i>1 stuffing=zeros([i-1,1]); else stuffing=[];end
    index_of_links_temp=[boolean(stuffing);...dist_to_all_TEMP > 0 & ... 
                        (dist_to_all_TEMP < maxDistConstant & ...
                         norm_ang_to_all_TEMP < maxAngleConstant & ...
                         dist_plane_to_points < maxDist2PlaneConstant)&...
                         (anglebetweenQs(:,1)<maxQConstant(1)&...
                          anglebetweenQs(:,2)<maxQConstant(2)&...
                          anglebetweenQs(:,3)<maxQConstant(3)&...
                          anglebetweenQs(:,4)<maxQConstant(4)&...
                          anglebetweenQs(:,5)<maxQConstant(5)&...
                          anglebetweenQs(:,6)<maxQConstant(6))]; %find if any angles are bad, if not then its ok

	%updates connectivity from node i (make sure there is no self links)
    connectivityGraph(index_of_links_temp,i)=1;
    connectivityGraph(i,index_of_links_temp)=1;    
    connectivityGraph(i,i)=0;
    
    %dist constraint    
    index_of_links_temp=[boolean(stuffing);(dist_to_all_TEMP < maxDistConstant)];
    connectivityGraph_C1(index_of_links_temp,i)=1;connectivityGraph_C1(i,index_of_links_temp)=1; 
    %angle to norm    
    index_of_links_temp=[boolean(stuffing);(norm_ang_to_all_TEMP < maxAngleConstant)];
    connectivityGraph_C2(index_of_links_temp,i)=1;connectivityGraph_C2(i,index_of_links_temp)=1; 
    %perp dist to plane
    index_of_links_temp=[boolean(stuffing);(dist_plane_to_points < maxDist2PlaneConstant)];
    connectivityGraph_C3(index_of_links_temp,i)=1;connectivityGraph_C3(i,index_of_links_temp)=1; 
    %joints
    index_of_links_temp=[boolean(stuffing);(anglebetweenQs(:,1)<maxQConstant(1)&anglebetweenQs(:,2)<maxQConstant(2)&...
                          anglebetweenQs(:,3)<maxQConstant(3)&anglebetweenQs(:,4)<maxQConstant(4)&...
                          anglebetweenQs(:,5)<maxQConstant(5)&anglebetweenQs(:,6)<maxQConstant(6))];
    connectivityGraph_C4(index_of_links_temp,i)=1;connectivityGraph_C4(i,index_of_links_temp)=1; 

        
    
end

if plotconnectivityGraphs
    figure
    subplot(2,2,1); imshow(abs(connectivityGraph_C1-1));title('Dist Constraint')
    subplot(2,2,2); imshow(abs(connectivityGraph_C2-1));title('Angle to Norm. Constraint')
    subplot(2,2,3); imshow(abs(connectivityGraph_C3-1));title('Dist to Plane Constraint')
    subplot(2,2,4); imshow(abs(connectivityGraph_C4-1));title('Joint Constraint')
end


%% Use bredthfirst tree searh, determine links between planes on surface
% $$ \begin{array}{cccccc} 
% \mbox{Connectivity Graph between planes}\\
% & 1& 2 & 3 & 4 & etc\\
% 1& 0 & 1 & 1 & 0 & etc\\
% 2& 1 & 0 & 1 & 0 & etc\\
% 3& 1 & 1 & 0 & 1 & etc\\
% 4& 0 & 0 & 1 & 0 & etc\\
% etc& etc & etc & etc & etc & 0\\
% \end{array}$$
%%
% We are looking for clusters in the graph which represent connectivity and
% hence a larger surface made up of the planes

% look for hubs as with network theory
[numlinks,popularIndex]=sort(sum(connectivityGraph,1),'descend');

% If any planes unregister search for clusters in the graph
while ~isempty(find(registered_to_surface==0,1))
    index_temp=popularIndex(find(registered_to_surface(popularIndex)==0,1));

    %check if the larger_surface variable exists
    if ~exist('larger_surface','var')
        larger_surface.originalnorm=all_norms(index_temp,:);
        larger_surface.originalcenter=all_centers(index_temp,:);
    else
        larger_surface(end+1).originalnorm=all_norms(index_temp,:);
        larger_surface(end).originalcenter=all_centers(index_temp,:);
    end
    curr_surf=size(larger_surface,2);   
    %register this plane to a surface
    registered_to_surface(index_temp)=curr_surf;
    %register all linked planes to the surface
    linked_planes_temp=find(connectivityGraph(:,index_temp)>0);
    if linked_planes_temp>0
        registered_to_surface(linked_planes_temp)=curr_surf;
        planes_to_check=linked_planes_temp;
        %go through until there are no more trees to check
        while size(planes_to_check,2)>0
            %find the links to this current plane node
            linked_planes_temp=find(connectivityGraph(planes_to_check(1),:)>0);            

            %make sure they are not registered to the surface already
            linked_planes_temp=setdiff(linked_planes_temp',...
                                       find(registered_to_surface==curr_surf));

            %update the list of registered to the surface planes
            registered_to_surface(linked_planes_temp)=curr_surf;

            % fill planes to check with new links, removing current checked plane
            planes_to_check=[planes_to_check(2:end);linked_planes_temp];
        end
    end
    larger_surface(end).registered_to_surface=find(registered_to_surface==curr_surf);
end
timetaken = toc
% profile off; profile viewer;

% find surfaces that have more than a single plane
validnewsurfaces=[];
for i=1:size(larger_surface,2)
    if size(larger_surface(i).registered_to_surface,1)>=minplanes2callAsurface
        validnewsurfaces=[validnewsurfaces,i];
    end
end


if plotconnectivityGraphs
    connectivityGraph_inv=abs(connectivityGraph-1);
    figure;
    % subplot(1,2,1)
    imshow(connectivityGraph_inv);
    title('Combined Connectivity Graph')
    for i=1:size(registered_to_surface,1)%size(connectivityGraph_inv,1);
        if ~isempty(find(registered_to_surface(i)==validnewsurfaces,1))
            connectivityGraph_inv(connectivityGraph_inv(:,i)==0,i)=registered_to_surface(i)+1;
        end
    end
    % %make all invalid surfaces or non connects back to 0
    % connectivityGraph_inv(find(connectivityGraph_inv<=1))=0;
    % %All the rest get scalled from 0 to 1
    % connectivityGraph_inv=(connectivityGraph_inv)/max(max(connectivityGraph_inv));
    % %inver again so we have 0 to 1(for no connects)
    % connectivityGraph_inv=abs(connectivityGraph_inv-1);
    % subplot(1,2,2)
    % imshow(connectivityGraph_inv)

    %make a matrix of the same size
    connectivityGraph_inv_temp=connectivityGraph_inv;
    figure
    sqrtedsize=ceil(sqrt(size(validnewsurfaces,2)))
    for i=1:size(validnewsurfaces,2)
        %set it all to 1s (eg all white)
        connectivityGraph_inv_temp(:,:)=1;
        subplot(sqrtedsize,sqrtedsize,i)
        title(['Showing connectivity of surface',num2str(i)]);
        connectivityGraph_inv_temp(find(connectivityGraph_inv==validnewsurfaces(i)+1))=0;
        imshow(connectivityGraph_inv_temp);
    end
end

%% Draw the center of all the tiles
if draw_center_of_all_tiles
    figure;
    hold on;
    title (['Number of surfaces found = ',num2str(size(validnewsurfaces,2)),'. Time Taken: ',num2str(timetaken)])


    colorindex=1;
    textforlegend=[];

    for i=validnewsurfaces
        if mod(i,4)==1; colorval=rand*([colorindex/size(validnewsurfaces,2),0,0]);
        elseif mod(i,4)==2; colorval=rand*([1,colorindex/size(validnewsurfaces,2),0]);
        elseif mod(i,4)==3; colorval=rand*([colorindex/size(validnewsurfaces,2),colorindex/size(validnewsurfaces,2),0]);
        else; colorval=[0,0,colorindex/size(validnewsurfaces,2)];
        end    
        plot3(all_centers(larger_surface(i).registered_to_surface,1),...
              all_centers(larger_surface(i).registered_to_surface,2),...          
               all_centers(larger_surface(i).registered_to_surface,3),'linestyle','none','color',colorval,'marker','*');             
           textforlegend{colorindex}=['Plane ',num2str(colorindex)];
        text(larger_surface(i).originalcenter(1)*1.1,larger_surface(i).originalcenter(2)*1.1,larger_surface(i).originalcenter(3)*1.1,num2str(colorindex),'FontSize',18)
        colorindex=colorindex+1;
    end
    legend(textforlegend);
    try plot(r,Q,'axis',gcf);light;end;
    axis equal;
    view(3)
end

%shows the distribution of the surfaces
if showClusterNormalDist
    for i=validnewsurfaces
        norm_ang_to_all_TEMP=ones([size(larger_surface(i).registered_to_surface,1),size(larger_surface(i).registered_to_surface,1)]);
        for j=1:size(larger_surface(i).registered_to_surface,1)-1
            norm_ang_to_all_TEMP(j+1:size(larger_surface(i).registered_to_surface,1),j)=dot([all_norms(larger_surface(i).registered_to_surface(j),1)*ones([size(larger_surface(i).registered_to_surface(j+1:end),1),1]),...
                                      all_norms(larger_surface(i).registered_to_surface(j),2)*ones([size(larger_surface(i).registered_to_surface(j+1:end),1),1]),...
                                      all_norms(larger_surface(i).registered_to_surface(j),3)*ones([size(larger_surface(i).registered_to_surface(j+1:end),1),1])],...
                                      all_norms(larger_surface(i).registered_to_surface(j+1:end),:),2);                          
            norm_ang_to_all_TEMP(j,j+1:size(larger_surface(i).registered_to_surface,1))=norm_ang_to_all_TEMP(j+1:size(larger_surface(i).registered_to_surface,1),j)';                          
        end
        norm_ang_to_all_TEMP=acos(norm_ang_to_all_TEMP);
        norm_ang_to_all_TEMP(norm_ang_to_all_TEMP>pi/2)=pi-norm_ang_to_all_TEMP(norm_ang_to_all_TEMP>pi/2);
        figure(2);
        surf(norm_ang_to_all_TEMP*180/pi)
        pause(5)
    end
end

% save to file
save('planeseachoutcome.mat','larger_surface','plane');

%% FUNCTION: dis_bet_plane_n_pnt_internal
% $$ \begin{array}{l}
% \mbox{Point(s) passed in... } P\\
% \mbox{Distances: (1 or many)... } dis_{to\_plane}=\left|\frac{AP_x + BP_y + CP_z + D}{\sqrt{A^2+B^2+C^2}}\right|
% \end{array}$$

% Pass in the ABCD of a plane and a set of points and  this function will return
% the distance between them
function dis_to_plane=dis_bet_plane_n_pnt_internal(plane_equ,pnt)
%dis_to_plane=zeros([1,size(pnt,1)]);
if size(pnt,1)>1
        dis_to_plane=abs((plane_equ(1)*pnt(:,1)+...
                                       plane_equ(2)*pnt(:,2)+...
                                       plane_equ(3)*pnt(:,3)+plane_equ(4))./...
                                       sqrt(plane_equ(1)^2+plane_equ(2)^2+plane_equ(3)^2));
                                   
else
    dis_to_plane=abs((plane_equ(1)*pnt(1)+plane_equ(2)*pnt(2)+plane_equ(3)*pnt(3)+plane_equ(4))/...
             sqrt(plane_equ(1)^2+plane_equ(2)^2+plane_equ(3)^2));
end