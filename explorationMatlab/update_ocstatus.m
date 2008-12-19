%function for updating the voxels status
%pass in a block of classified data that corresponds with the last scans
%PointData and RangeData

% should return nothing

function update_ocstatus(ClassifiedData)

global workspace PointData RangeData robmap_h
%get the variables from the workspace 
class_cubesize=workspace.class_cubesize;
minclassifications=workspace.minclassifications;
classfierthreshhold=workspace.classfierthreshhold;

pointswithclass=zeros([size(PointData,1)*size(PointData,2),4]);
CorrespondingRange=zeros([size(PointData,1)*size(PointData,2),1]);

for i=1:min(size(PointData,1),size(ClassifiedData,1))
    pointswithclass((i-1)*size(PointData,2)+1:i*size(PointData,2),:)=[squeeze(PointData(i,:,:)),ClassifiedData(i,:)'];
    CorrespondingRange((i-1)*size(PointData,2)+1:i*size(PointData,2),:)=RangeData(i,:);
end;

%only use the points that are greater than 0 range since the others make no
%sense
pointswithclass=pointswithclass(CorrespondingRange>20,:);

%discreatise into grid
class_ocgrid=[round(pointswithclass(:,1:3)/class_cubesize),pointswithclass(:,4)];
[level1,level2]=GetImpLevInfo(class_ocgrid(:,1:3)*class_cubesize);
class_ocgrid=class_ocgrid(level2,:);

%% Add bayesian update
load Classification_Criteria.mat

%how many materials not including unknown
% nummaterial= size(class_ocgrid,2)-1;
%all which have a MEAN and STD in the HParams which is all except edge/unknowns
nummaterial=size(struct2cell(HParas.I),1)/2;

uniformdisOnMats=ones([1,nummaterial])/nummaterial;
%make sure the structure is valid
if ~isfield(workspace,'probofmaterial');
    workspace.probofmaterial=[];
end

%default is for it to be false
startnewVoxel=false;

for i=1:size(class_ocgrid,1)
    cur_mat=class_ocgrid(i,4);
    %must be within actual materials not unknown or edges
    if cur_mat<=nummaterial
        %if there is nothing in the probofmaterial
        if ~isempty(workspace.probofmaterial)
            curr_voxel=find(workspace.probofmaterial(:,1)==class_ocgrid(i,1) &...
                            workspace.probofmaterial(:,2)==class_ocgrid(i,2) & ...
                            workspace.probofmaterial(:,3)==class_ocgrid(i,3),1);
            if ~isempty(curr_voxel)
                %all others decreased by prob of 0.05
                workspace.probofmaterial(curr_voxel,[4:3+cur_mat-1,3+cur_mat+1:end])=...
                workspace.probofmaterial(curr_voxel,[4:3+cur_mat-1,3+cur_mat+1:end])*(1-workspace.classifyConfidence);
                %Actual is increased by workspace.classifyConfidence
                workspace.probofmaterial(curr_voxel,3+cur_mat)=workspace.probofmaterial(curr_voxel,3+cur_mat)*workspace.classifyConfidence;
                
                %normalise probability
                                     normalisingfactor=sum(workspace.probofmaterial(curr_voxel,4:end));
                workspace.probofmaterial(curr_voxel,4:end)=workspace.probofmaterial(curr_voxel,4:end)/normalisingfactor;

            else
                startnewVoxel=true;
            end
        else
            startnewVoxel=true;
        end
        
        %if the probofmaterial variable is empty or there was no valid
        %voxel start a new line
        if startnewVoxel
            workspace.probofmaterial(end+1,:)=[class_ocgrid(i,1:3), uniformdisOnMats];
            %all others decreased by prob of 0.05
            workspace.probofmaterial(end,[4:3+cur_mat-1,3+cur_mat+1:end])=...
            workspace.probofmaterial(end,[4:3+cur_mat-1,3+cur_mat+1:end])*(1-workspace.classifyConfidence);
            %Actual is increased by workspace.classifyConfidence
            workspace.probofmaterial(end,3+cur_mat)=workspace.probofmaterial(end,3+cur_mat)*workspace.classifyConfidence;            
            
            %normalise probability
                          normalisingfactor=sum(workspace.probofmaterial(end,4:end));
            workspace.probofmaterial(end,4:end)=workspace.probofmaterial(end,4:end)/normalisingfactor;
        end
        
        startnewVoxel=false;
    end
end


try     
    aabb = [workspace.min; workspace.max];
    % aabb = [-inf, -inf, -inf; inf, inf, inf];
    hMesh = robmap_h.Mesh(aabb);
    % f = hMesh.FaceData;
    v = hMesh.VertexData;
    [level1,level2]=GetImpLevInfo(v);
    v_decrete=unique(round(v(level2,:)/class_cubesize),'rows');
    [vals,indexa,indexb]=intersect(v_decrete,workspace.probofmaterial(:,1:3),'rows');
    workspace.probofmaterial=workspace.probofmaterial(indexb,:);
end

return

% workspace.ALLlastscandataInWkspace(:,1)


%%
% hold on;
% try planeplotHa(end+1)=plot3(class_ocgrid(:,1)*class_cubesize,class_ocgrid(:,2)*class_cubesize,class_ocgrid(:,3)*class_cubesize,'y','marker','.','markersize',0.1,'linestyle','none');end


% pnts=[-7,10,26;-9,3,30;-8,-13,27];
% pause
% for i=1:3
%     pnt=pnts(i,:);
%     text(pnt(1)*class_cubesize,pnt(2)*class_cubesize,pnt(3)*class_cubesize,num2str(i));
%     figure;hist(class_ocgrid(find(class_ocgrid(:,1)==pnt(1) & class_ocgrid(:,2)==pnt(2) & class_ocgrid(:,3)==pnt(3)),4))
%     pause
% end

%% Update the ocgrid structure, NOT REALLY NEEDED NOW
%make sure the structure is valid
if ~isfield(workspace,'ocgrid');
    workspace.ocgrid=[];
end
if size(workspace.ocgrid,2)~=6
    tempocgrid=unique(class_ocgrid(:,1:3),'rows');   
else
    tempocgrid=setdiff(class_ocgrid(:,1:3),workspace.ocgrid(:,1:3),'rows');
end

workspace.ocgrid=[workspace.ocgrid;tempocgrid,zeros([size(tempocgrid,1),3])];
[level1,level2]=GetImpLevInfo(workspace.ocgrid(:,1:3)*class_cubesize);
workspace.ocgrid=workspace.ocgrid(level2,:);
newoc_toupdate=unique(class_ocgrid(:,1:3),'rows');
[vals,indexa,indexb]=intersect(newoc_toupdate,workspace.ocgrid(:,1:3),'rows');

for i=1:indexb
    tempdata=class_ocgrid(class_ocgrid(:,1)==workspace.ocgrid(i,1) &...
                          class_ocgrid(:,2)==workspace.ocgrid(i,2) & ...
                          class_ocgrid(:,3)==workspace.ocgrid(i,3),4);
    metalnum=size(find(tempdata<=5),1);
    woodnum=size(find(tempdata>=6 & tempdata<=7),1);
    unknownnum=size(find(tempdata>=8),1);
    workspace.ocgrid(i,4)=workspace.ocgrid(i,4)+metalnum;
    workspace.ocgrid(i,5)=workspace.ocgrid(i,5)+woodnum;
    workspace.ocgrid(i,6)=workspace.ocgrid(i,6)+unknownnum;
end

%% we only want to hold the voxels which correspond with the current map
try     
    aabb = [-1.5, -1.5, 0.2; 1.5, 1.5, 1.8];
    % aabb = [-inf, -inf, -inf; inf, inf, inf];
    hMesh = robmap_h.Mesh(aabb);
    % f = hMesh.FaceData;
    v = hMesh.VertexData;
    [level1,level2]=GetImpLevInfo(v);
    v_decrete=unique(round(v(level2,:)/class_cubesize),'rows');
    [vals,indexa,indexb]=intersect(v_decrete,workspace.ocgrid(:,1:3),'rows');
    workspace.ocgrid=workspace.ocgrid(indexb,:);
end


% figure(2)
sumofclass=workspace.ocgrid(:,4)+workspace.ocgrid(:,5);
warning('off','MATLAB:divideByZero')
% try classifiedvoxels=find(sumofclass>=minclassifications &...
%                      (workspace.ocgrid(:,4)./workspace.ocgrid(:,5)>classfierthreshhold |...
%                       workspace.ocgrid(:,5)./workspace.ocgrid(:,4)>classfierthreshhold));end
try UNclassifiedvoxels=find(sumofclass<minclassifications | ...
                     (workspace.ocgrid(:,4)./workspace.ocgrid(:,5)<=classfierthreshhold &...
                      workspace.ocgrid(:,5)./workspace.ocgrid(:,4)<=classfierthreshhold));end
warning('on','MATLAB:divideByZero')
% if show_classified
%     hold on;
%     try planeplotHa(end+1)=plot3(workspace.ocgrid(UNclassifiedvoxels,1)*class_cubesize,...
%           workspace.ocgrid(UNclassifiedvoxels,2)*class_cubesize,...
%           workspace.ocgrid(UNclassifiedvoxels,3)*class_cubesize,'y','marker','.','markersize',0.5,'linestyle','none');end
%     %plot metal and wood voxels
% 
%     metalvoxels=workspace.ocgrid(classifiedvoxels(workspace.ocgrid(classifiedvoxels,4)>workspace.ocgrid(classifiedvoxels,5)),1:3);
%     if size(metalvoxels,1)>0
%         try planeplotHa(end+1)=plot3(metalvoxels(:,1)*class_cubesize,metalvoxels(:,2)*class_cubesize,metalvoxels(:,3)*class_cubesize,'r.');end
%     end
% 
%     woodvoxels=workspace.ocgrid(classifiedvoxels(workspace.ocgrid(classifiedvoxels,4)<workspace.ocgrid(classifiedvoxels,5)),1:3);
%     if size(woodvoxels,1)>0
%         try planeplotHa(end+1)=plot3(woodvoxels(:,1)*class_cubesize,woodvoxels(:,2)*class_cubesize,woodvoxels(:,3)*class_cubesize,'b.');end
%     end
% 
%     drawnow
% end




