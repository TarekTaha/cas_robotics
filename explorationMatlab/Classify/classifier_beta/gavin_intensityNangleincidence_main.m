%% gavin_intensityNangleincidence
%
% *Description:* This will go through a ply file which has boxes around
% materials of interest. It can be used for training a classifier model
% purposes 
%

function [all_data,all_data_T,all_data_xyz]=gavin_intensityNangleincidence_main(filename)

close all;

%if there is no filename of the model passed in
if nargin==0
    filename='latest model';
    warning('Please change the model filename so it wont be overwritten')
end

    
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
plot_config.show_normfit=false;
% mirror the points about 0'? (%redundant)
plot_config.addinmirror=false;

% Config
global config
% the minimum ray to use (min ==1)
config.minray=100;%1;%130;%30;
% the max ray to use (max==502)
config.maxray=400;%502;%300;%440;
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
classifierdata(1).xyz=[];classifierdata(2).xyz=[];classifierdata(3).xyz=[];classifierdata(4).xyz=[];

%% %I KNOW THIS SHOULD BE IN A FOR LOOP BUT THE PARAEMTERS OF THE BOXES MAKE IT TOO HARD

%The bounding boxes worked out by Steve
if grid_file_number(1)==true
    xshift=0;
    fileName = 'grid_1.ply'; % looking at wood (was grid_2)
    display('Scanner at wood');
    %NOTE THE X VALUE IS ONLY VALID FOR EACH SCAN AND IS NOT AN ABSOLUTE
    %TRUTH
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
    updateData(voxelsize,box,source,intestity,angleincidence,scanrange,vertices,xshift)

end

if grid_file_number(3)==true   
    %the x shift from the first scan
    xshift=1.34;
    fileName = 'grid_3.ply';
    display('Scanner at rusty metal');
    
    %NOTE THE X VALUE IS ONLY VALID FOR EACH SCAN AND IS NOT AN ABSOLUTE
    %TRUTH
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
    updateData(voxelsize,box,source,intestity,angleincidence,scanrange,vertices,xshift)

end

if grid_file_number(4)==true   
    %the x shift from the first scan
    xshift=+1.34-2.5;
    fileName = 'grid_4.ply'; % looking at shiny metal
    display('Scanner at shiny metal');
    %NOTE THE X VALUE IS ONLY VALID FOR EACH SCAN AND IS NOT AN ABSOLUTE
    %TRUTH
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
    updateData(voxelsize,box,source,intestity,angleincidence,scanrange,vertices,xshift)
end

%% put in correct format
all_data=[];
all_data_T=[];
all_data_xyz=[];
for material_num=1:size(classifierdata,2)
    all_data=[all_data;classifierdata(material_num).val];
    all_data_T=[all_data_T;material_num* ones([size(classifierdata(material_num).val,1),1])];
    all_data_xyz=[all_data_xyz;classifierdata(material_num).xyz];
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

save([filename,'_DATA.mat'],'all_data','all_data_T','all_data_xyz');

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