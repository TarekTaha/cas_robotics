%% gavin_intensityNangleincidence
%
% *Description:* This will go through a ply file which has boxes around
% materials of interest. It can be used for training a classifier model
% purposes 
%

function [all_data,all_data_T]=gavin_intensityNangleincidence()

close all;
    
% classifierdata holds for each material group & for every point in the form>>
% classifierdata(materialtype_num).val=[angle_of_inc, intensity, range]
global classifierdata;

% Plotting Configs
global plot_config 
% show the angle of incidence?
plot_config.show_angleofincidenceplot=true;
% show the polyfit for each range slice?
plot_config.show_polyfit=false;
% discretise the range and angle then fit a norm prob dist to the intensity 
plot_config.show_normfit=true;
% mirror the points about 0'? (%redundant)
plot_config.addinmirror=false;

% Config
global config
% the minimum ray to use
config.minray=130;%30;
% the max ray to use
config.maxray=300;%440;
% get the pose from out of the ply file (%redundant)
config.getposedata=false; if config.getposedata setuprobot(); end
% for each voxel display which material box it is in
config.show_voxelpostiioninfo=false;
% Collect all wood together as one material
config.allwood_thesame=true;


% Which grid files to use? (note there is no  grid 2)
grid_file_number=[true,false,true,true];

% The size of voxels, this is not intuitively altered as the dimension
% changes so it is constantly like this. Only worthwhile changing
% individual values if we only have one wall in one dimension
voxelsize=[0.1,0.1,0.1];

% The size of disks for surface making. If too large there will be problems
% with the angle of incidence at the corners 
mu=0.15;

% The material type names
materialtype{1}='rusted metal';
materialtype{2}='wood';
materialtype{3}='wood on roof'; %note: if allwood_thesame==true, this will actually be 2 as well
materialtype{4}='shiny metal';  %note: if allwood_thesame==true, this will be 3

% Clear values
classifierdata(1).val=[];classifierdata(2).val=[];classifierdata(3).val=[];classifierdata(4).val=[];

%% %I KNOW THIS SHOULD BE IN A FOR LOOP BUT THE PARAEMTERS OF THE BOXES MAKE IT TOO HARD

%The bounding boxes worked out by Steve
if grid_file_number(1)==true
    fileName = 'grid_1.ply'; % looking at wood (was grid_2)
    display('Scanner at wood');
    % Metal
    box(1).lower = [1.1 -.6 1.2];
    box(1).upper = [2.1 -.4 1.45];
    % Wood
    box(2).lower = [-0.07 -.6 1.0];
    box(2).upper = [1.07 -.4 1.4];
    % Wood (ceiling)
    box(3).lower = [-0.5 -0.5 1.4];
    box(3).upper = [1.5 0.5 1.8];
    %shinny metal (matt finish)
    box(4).lower = [0 0 0];
    box(4).upper = [0 0 0];
    
    % call the file to get data
    [vertices,normals,scanrange,source,intestity,positiondata]=getdata(fileName);
    [vertices,source,intestity,angleincidence,scanrange]=getangle(mu,vertices,normals,scanrange,source,intestity,positiondata);    
    updateData(voxelsize,box,source,intestity,angleincidence,scanrange,vertices)

end

if grid_file_number(3)==true   
    fileName = 'grid_3.ply';
    display('Scanner at rusty metal');
    % Metal
    box(1).lower = [-0.1 -.7 1.1];
    box(1).upper = [1.3 -.4 1.4];
    % Wood
    box(2).lower = [-1.3 -.7 1.1];
    box(2).upper = [-0.1 -.4 1.4];
    % Wood
    box(3).lower = [-1.5 -0.6 1.4];
    box(3).upper = [1.5 0.6 1.8];
    %shinny metal (matt finish)
    box(4).lower = [0 0 0];
    box(4).upper = [0 0 0];
    
    % call the file to get data
    [vertices,normals,scanrange,source,intestity,positiondata]=getdata(fileName);
    [vertices,source,intestity,angleincidence,scanrange]=getangle(mu,vertices,normals,scanrange,source,intestity,positiondata);    
    updateData(voxelsize,box,source,intestity,angleincidence,scanrange,vertices)

end

if grid_file_number(4)==true   
    fileName = 'grid_4.ply'; % looking at shiny metal
    display('Scanner at shiny metal');
    % Metal display('note how this is turned off');
    box(1).lower = [0 0 0];
    box(1).upper = [0 0 0];
    % Wood
    box(2).lower = [1.1 -.6 1.0];
    box(2).upper = [2.2 -.4 1.4];
    % Wood
    box(3).lower = [-0.5 -0.5 1.4];
    box(3).upper = [1.5 0.5 1.8];
    %shinny metal (matt finish)
    box(4).lower = [0 -.6 1.0];        
    box(4).upper = [1.1 -.4 1.4];
    [vertices,normals,scanrange,source,intestity,positiondata]=getdata(fileName);
    [vertices,source,intestity,angleincidence,scanrange]=getangle(mu,vertices,normals,scanrange,source,intestity,positiondata);    
    updateData(voxelsize,box,source,intestity,angleincidence,scanrange,vertices)

end

%% put in correct format
all_data=[];
all_data_T=[];
for material_num=1:size(classifierdata,2)
    all_data=[all_data;classifierdata(material_num).val];
    all_data_T=[all_data_T;material_num* ones([size(classifierdata(material_num).val,1),1])];
end

all_data=all_data;
all_data_T=all_data_T;

%% if all wood is the same, materials 2&3->2 and 4->3 
if config.allwood_thesame
    %change all mat 3 to mat 2 and mat 4 to mat 3
    all_data_T(all_data_T==3)=2;
    all_data_T(all_data_T==4)=3;
    
    %recreate classifierdata
    temp=classifierdata;
    classifierdata=[];
    classifierdata(1).val=temp(1).val;
    classifierdata(2).val=[temp(2).val;temp(3).val];
    classifierdata(3).val=temp(4).val;
end

    
%% if plotting is required
if plot_config.show_angleofincidenceplot
    for materialtype_num=1:size(classifierdata,2)  
        figure(materialtype_num)
        title(materialtype{materialtype_num});
        hold on;
        xlabel('angle of incidence')
        ylabel('intensity')
        zlabel('range');
        
        plot3(classifierdata(materialtype_num).val(:,1),...
              classifierdata(materialtype_num).val(:,2),...
              classifierdata(materialtype_num).val(:,3),'r.');                          
        axis([min(classifierdata(materialtype_num).val(:,1)),max(classifierdata(materialtype_num).val(:,1)),...
            0,max(classifierdata(materialtype_num).val(:,2)),...
            0,max(classifierdata(materialtype_num).val(:,3))]);
    end
end 

%if you want to fit liens or normal fit to data
if plot_config.show_normfit || plot_config.show_polyfit
    gavin_fitlines(classifierdata,plot_config.show_normfit,plot_config.show_polyfit)
end

end

%% getdata
% *Description:* 
% 1) Opens the file required, 
% 2) Works out the surfaces due to the verticy data with PCA
% 3) Angle on incidence of each point with the surface
% 4) Updates classifierdata with a new point (angle, intensity, range)
function [vertices,normals,scanrange,source,intestity,positiondata]=getdata(fileName)

global r config;
hMesh = actxserver('EyeInHand.TriangleMesh');
hMesh.AddRangeGrid([pwd, '\', fileName]);   
positiondata = plyread(fileName);

% get last joint of robot position  
if config.getposedata
    Q=getjointfromfile(fileName);
    Tr=fkine(r,Q);
end



% get data from hmesh
vertices  = hMesh.VertexData;
normals   = hMesh.NormalData;
scanrange = hMesh.ScannerData(:,1);
intestity = hMesh.ScannerData(:,2);
autogain  = hMesh.ScannerData(:,3);
intestity = intestity./autogain;
source    = hMesh.ScannerData;
end

%% getangle
% *Description:* 
% 1) Works out the surfaces due to the verticy data with PCA
% 2) Angle on incidence of each point with the surface
function [vertices,source,intestity,angleincidence,scanrange]=getangle(mu,vertices,normals,scanrange,source,intestity,positiondata)

global config

% Min and max ray as requested  
minray=config.minray;
maxray=config.maxray;

angleincidence=inf*ones([size(vertices,1),1]);
    
%make surfaces and collect all homepoints and normals
tic;surface_making_simple(vertices,mu);toc
global plane
plane_homepoints=zeros([size(plane,2),3]);
plane_normals=zeros([size(plane,2),3]);
plane_for_vertices=zeros([size(vertices,1),1]);

%extract info from planes make sure normals are correct way
for i=1:size(plane,2)
    plane_homepoints(i,:)=plane(i).home_point;
    %get a vector from plane a one from averaging 
    vec1=plane(i).home_point-plane(i).normal_by_eigenval';
    vec1=vec1/norm(vec1);      
    %average across the normals to see which direction it is in
    vec2=plane(i).home_point-sum(normals(plane(i).points,:),1)/size(plane(i).points,1);
    vec2=vec2/norm(vec2);
    % they should be in the same direction
    if acos(dot(vec1,vec2))<pi/2
        plane_normals(i,:)=plane(i).normal_by_eigenval';
    else %need to reverse the direction of the normal
        plane_normals(i,:)=-plane(i).normal_by_eigenval';
    end
    %register each vertciy to a plane which holds is 
    plane_for_vertices(plane(i).points)=i;
end    

    %make sure all vertices have plane if not put in a closest plane
    for i=find(plane_for_vertices==0)'
        [nothing, index]=min(sum((plane_for_vertices(i)-plane_homepoints).^2,2));
        plane_for_vertices(i)=index;
    end
    
    %determine angle of incidence to plane for each point
    for i=1:max(source(:,6))
        scanorigin(i).positiondata=[positiondata.motion.origin_x_first(i)+positiondata.motion.approach_x_last(i),...
                                    positiondata.motion.origin_y_first(i)+positiondata.motion.approach_y_last(i),...
                                    positiondata.motion.origin_z_first(i)+positiondata.motion.approach_z_last(i)]/2;                            
        touse=(source(:,6)==i & source(:,5)>minray & source(:,5)<maxray);
        %         tempvector=[vertices(touse,1)-scanorigin(i).val(1),vertices(touse,2)-scanorigin(i).val(2),vertices(touse,3)-scanorigin(i).val(3)];
        tempvector=[vertices(touse,1)-scanorigin(i).positiondata(1),vertices(touse,2)-scanorigin(i).positiondata(2),vertices(touse,3)-scanorigin(i).positiondata(3)];            
        tempvector=[tempvector(:,1)./sqrt(sum(tempvector.^2,2)),tempvector(:,2)./sqrt(sum(tempvector.^2,2)),tempvector(:,3)./sqrt(sum(tempvector.^2,2))];
        angleincidence(touse)=acos(dot(tempvector,plane_normals(plane_for_vertices(touse),:),2))*180/pi;
    end

     %resize all to remove crap from outskirts    
    vertices=vertices(angleincidence<inf,:);
    source=source(angleincidence<inf,:);
    intestity=intestity(angleincidence<inf,:);
    angleincidence=angleincidence(angleincidence<inf,:);
    scanrange=scanrange(angleincidence<inf);
end

%% updateData
% 1) Updates classifierdata with a new point (angle, intensity, range)

function updateData(voxelsize,box,source,intestity,angleincidence,scanrange,vertices)

global plot_config classifierdata config
% !!!Note sure if this is a good idea!!!
%fold about 90 since we can't have it greater than this
angleincidence(angleincidence>90)=180-angleincidence(angleincidence>90);
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

%discretise space into voxels
voxelindex=[round(vertices(:,1)/voxelsize(1)),round(vertices(:,2)/voxelsize(2)),round(vertices(:,3)/voxelsize(3))];
uniquevoxels=unique(voxelindex,'rows');

m1_l=round(box(1).lower./voxelsize);
m1_u=round(box(1).upper./voxelsize);
w1_l=round(box(2).lower./voxelsize);
w1_u=round(box(2).upper./voxelsize);
w2_l=round(box(3).lower./voxelsize);
w2_u=round(box(3).upper./voxelsize);
m2_l=round(box(4).lower./voxelsize);
m2_u=round(box(4).upper./voxelsize);

%find the counts for the unique ones
for i=1:size(uniquevoxels,1)
    if uniquevoxels(i,1)>=m1_l(1) && uniquevoxels(i,1)<=m1_u(1) &&...
       uniquevoxels(i,2)>=m1_l(2) && uniquevoxels(i,2)<=m1_u(2) &&...
       uniquevoxels(i,3)>=m1_l(3) && uniquevoxels(i,3)<=m1_u(3)
        materialtype='rusted metal';
        materialtype_num=1;
    elseif uniquevoxels(i,1)>=w1_l(1) && uniquevoxels(i,1)<=w1_u(1) &&...
           uniquevoxels(i,2)>=w1_l(2) && uniquevoxels(i,2)<=w1_u(2) &&...
           uniquevoxels(i,3)>=w1_l(3) && uniquevoxels(i,3)<=w1_u(3)
        materialtype='wood';
        materialtype_num=2;
    elseif uniquevoxels(i,1)>=w2_l(1) && uniquevoxels(i,1)<=w2_u(1) &&...
           uniquevoxels(i,2)>=w2_l(2) && uniquevoxels(i,2)<=w2_u(2) &&...
           uniquevoxels(i,3)>=w2_l(3) && uniquevoxels(i,3)<=w2_u(3)
        materialtype='wood on roof';
        materialtype_num=3;
    elseif uniquevoxels(i,1)>=m2_l(1) && uniquevoxels(i,1)<=m2_u(1) &&...
       uniquevoxels(i,2)>=m2_l(2) && uniquevoxels(i,2)<=m2_u(2) &&...
       uniquevoxels(i,3)>=m2_l(3) && uniquevoxels(i,3)<=m2_u(3)
        materialtype='shiny metal';            
        materialtype_num=4;
    else
        materialtype='UNKNOWN'; %#ok<NASGU>
        continue
    end

    if config.show_voxelpostiioninfo
        display(['this is a ', materialtype,'-type voxel']);        
    end
    verticesinvoxel=find(voxelindex(:,1)==uniquevoxels(i,1) & voxelindex(:,2)==uniquevoxels(i,2) & voxelindex(:,3)==uniquevoxels(i,3));

    %want to add the range data as another variable
    increment=0.1;
    for range=increment:increment:max(source(:,1))                      

        index=verticesinvoxel(source(verticesinvoxel,1)>(range-increment) & source(verticesinvoxel,1)<range);
        if isempty(index)
            continue;
        end
        if plot_config.addinmirror %mirror about 0'
            angle_data=[angleincidence(index);0-angleincidence(index)];
            intensity_data=[intestity(index);intestity(index)];
            range_data=[scanrange(index);scanrange(index)];
        else %dont do the mirroring
            angle_data=angleincidence(index);
            intensity_data=intestity(index);
            range_data=scanrange(index);
        end

        classifierdata(materialtype_num).val=[classifierdata(materialtype_num).val;angle_data,intensity_data,range_data];        
    end
end
    
    display('finished on ply file');
    
    

end

%%
%to get the joint angles form inside the ply file
function Q=getjointfromfile(fileName)
    %read just the 4th line where the pose of the robot is
    fid=fopen(fileName);for i=1:4; lineoftext = fgetl(fid);end;fclose(fid)
    [nothing,str]=strtok(lineoftext, '(');
    str=str(2:end-1);
    for i=1:6
        [temp,str]=strtok(str(2:end), ',');
        Q(i)=str2double(temp);
    end
    Q=deg2rad(Q);
end

%%
% %find the origin of the scan
% function getpositionoldway()
%     for i=1:max(source(:,6))
%         touse=(source(:,6)==i);
%         numvals=size(find(touse),1);
%         if numvals>0
%             all_scanorigin_estimates=vertices(touse,:)+...
%                 [source(touse,1).*normals(touse,1),...
%                  source(touse,1).*normals(touse,2),...
%                  source(touse,1).*normals(touse,3)];
%              
%             scanorigin(i).val=sum(all_scanorigin_estimates)/numvals;                                  
%                                   
%             
%             %if show plots
%             if true             
%                 temphandle=plot3(all_scanorigin_estimates(:,1),all_scanorigin_estimates(:,2),all_scanorigin_estimates(:,3),'r.');
%                 temp_planhandles=plot_planes(plane(plane_for_vertices(touse)),mu);
%                 temp_points=plot3(vertices(touse,1),vertices(touse,2),vertices(touse,3),'g.');
%                 linenumbers=find(touse==1);
%                 for j=1:size(linenumbers,1)
%                     temp_line(j)=plot3([vertices(linenumbers(j),1),...
%                                         vertices(linenumbers(j),1)+source(linenumbers(j),1).*normals(linenumbers(j),1)],...                       
%                                         [vertices(linenumbers(j),2),...
%                                         vertices(linenumbers(j),2)+source(linenumbers(j),1).*normals(linenumbers(j),2)],...
%                                        [vertices(linenumbers(j),3),...
%                                        vertices(linenumbers(j),3)+source(linenumbers(j),1).*normals(linenumbers(j),3)]);
%                 end
%                 
%                 hold on;plot3(scanorigin(i).val(1),scanorigin(i).val(2),scanorigin(i).val(3),'b*');
%                 hold on;plot3(scanorigin(i).positiondata(1),scanorigin(i).positiondata(2),scanorigin(i).positiondata(3),'g*');
%                 pause(2);
%                 try delete(temphandle);end                
%                 try delete(temp_planhandles);end                
%                 try delete(temp_points);end 
%                 try for j=1:size(linenumbers,1); delete(temp_line(j));end;end
%             end       
%         else
%             scanorigin(i).val=[];
%         end
%     end
% end
%% Steves stuff
%     hMesh.Smooth(3);
%     tic;hMesh.SmoothNormals(100000);toc % radius 100mm disks

% % function showSourceData(filename, box)
%     hMesh = actxserver('EyeInHand.TriangleMesh');
%     hMesh.AddRangeGrid([pwd, '\', fileName]);
% %     vertices = hMesh.VertexData;
% %     normals = hMesh.NormalData;
% %     ptMask = ~((box(2).lower(1) > vertices(:,1)) | (vertices(:,1) > box(1).upper(1)) | ...
% %     (box(2).lower(2) > vertices(:,2)) | (vertices(:,2) > box(1).upper(2)) | ...
% %     (box(2).lower(3) > vertices(:,3)) | (vertices(:,3) > box(1).upper(3)));
% %     normalCov = cov(normals(ptMask,:));
% %     disp(det(normalCov))
%     hMesh.Smooth(3);
%     tic;hMesh.SmoothNormals(100000);toc % radius 100mm disks
% %     hMesh.SmoothNormals(4000); % 4000 mm^2 disks
%     vertices = hMesh.VertexData;
%     normals = hMesh.NormalData;
%     source = hMesh.ScannerData;
%     figure;
%     ptMask1 = ~((box(1).lower(1) > vertices(:,1)) | (vertices(:,1) > box(1).upper(1)) | ...
%     (box(1).lower(2) > vertices(:,2)) | (vertices(:,2) > box(1).upper(2)) | ...
%     (box(1).lower(3) > vertices(:,3)) | (vertices(:,3) > box(1).upper(3)));
%     plot3(rad2deg(source(ptMask1,4)), source(ptMask1,2)./source(ptMask1,3), source(ptMask1,1),'.r');
%     hold on;
% 
%     ptMask2 = ~((box(2).lower(1) > vertices(:,1)) | (vertices(:,1) > box(2).upper(1)) | ...
%     (box(2).lower(2) > vertices(:,2)) | (vertices(:,2) > box(2).upper(2)) | ...
%     (box(2).lower(3) > vertices(:,3)) | (vertices(:,3) > box(2).upper(3)));
%     plot3(rad2deg(source(ptMask2,4)), source(ptMask2,2)./source(ptMask2,3), source(ptMask2,1), '.g');
% 
% %     ptMask3 = ~((box(3).lower(1) > vertices(:,1)) | (vertices(:,1) > box(3).upper(1)) | ...
% %     (box(3).lower(2) > vertices(:,2)) | (vertices(:,2) > box(3).upper(2)) | ...
% %     (box(3).lower(3) > vertices(:,3)) | (vertices(:,3) > box(3).upper(3)));
% %     hold on;
% %     plot3(rad2deg(source(ptMask3,4)), source(ptMask3,3)*100, (source(ptMask3,1)), '.g');
%     xlabel('Angle of incidence (degrees)', 'FontSize', 20); ylabel('Recovered intensity', 'FontSize', 20); zlabel('Range to scanner', 'FontSize', 20); 
% %     title(sprintf('%s in box (%.2f,%.2f,%.2f to %.2f,%.2f,%.2f)', fileName, box(1).lower, box(2).upper), 'Interpreter', 'None', 'FontSize', 20);
%     title(description,  'FontSize', 20);
%     legend('Metal', 'Wood');
%     
%     % find the most perpendicular scan 
%     scanNumber = source(source(:,4)==min(source(ptMask1|ptMask2,4)),6);
% %     scanNumber = 110;
%     scanMask1 = source(:,6) == scanNumber(1);
%     rayNumber = source(scanMask1,5);
% %     plot3(rad2deg(source(scanMask1,4)), source(scanMask1,2)./source(scanMask1,3), source(scanMask1,1),'-c');
%     
%     figure;
%     plot(rayNumber, normalised(source(scanMask1,1)),'color', 'b')
%     hold on;
%     plot(rayNumber, normalised(source(scanMask1,3)),'color', 'c');
%     plot(rayNumber, normalised(source(scanMask1,4)), 'linestyle', '-',  'marker', '.', 'color', 'r');
% %     plot(normalised(vertices(scanMask1,1)),'color', 'g');
%     plot(rayNumber, normalised(vertices(scanMask1,2)),'color', 'g');
%     legend('Range', 'Autogain', 'Angle of incidence', 'Y-coordinate');
%     title(['Data from scan ', int2str(scanNumber(1))], 'FontSize', 16);
%     xlabel('Ray number', 'FontSize', 16);    ylabel('Normalised value', 'FontSize', 16);