%% updateData
% *Description*: This function will undata the data stored in
% classifierdata so that it is sorted across  each material type 
%
% 1) Updates classifierdata with a new point (angle, intensity, range)

function updateData(voxelsize,box,source,intestity,angleincidence,scanrange,vertices,xshift)

global plot_config classifierdata config
% !!!Note sure if this is a good idea!!!
%fold about 90 since we shouldn't have any angle greater than this
if size(find(angleincidence(angleincidence>90),1),1)>0
    display('Some angle greater than 90')
    keyboard
end
    
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
            vertices_data=[vertices(index,:);vertices(index,:)];
        else %dont do the mirroring
            angle_data=angleincidence(index);
            intensity_data=intestity(index);
            range_data=scanrange(index);
            vertices_data=vertices(index,:);
        end

        %shift the x data by a perscribed amount
        vertices_data(:,1)=vertices_data(:,1)+xshift;
        
        classifierdata(materialtype_num).val=[classifierdata(materialtype_num).val;angle_data,intensity_data,range_data];        
        classifierdata(materialtype_num).xyz=[classifierdata(materialtype_num).xyz;vertices_data];
    end
end
    
    display('finished on ply file');

end