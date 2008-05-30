%function for updating the voxels status
%pass in a block of classified data that corresponds with the last scans
%PointData and RangeData

% should return nothing

function UNclassifiedvoxels=update_ocstatus(ClassifiedData)

global workspace PointData RangeData robmap_h
%get the variables from the workspace 
class_cubesize=workspace.class_cubesize;
minclassifications=workspace.minclassifications;
classfierthreshhold=workspace.classfierthreshhold;

pointswithclass=zeros([size(PointData,1)*size(PointData,2),4]);
CorrespondingRange=zeros([size(PointData,1)*size(PointData,2),1]);

for i=1:size(PointData,1);
    pointswithclass((i-1)*size(PointData,2)+1:i*size(PointData,2),:)=[squeeze(PointData(i,:,:)),ClassifiedData(i,:)'];
    CorrespondingRange((i-1)*size(PointData,2)+1:i*size(PointData,2),:)=RangeData(i,:);
end;

%only use the points that are greater than 0 range since the others make no
%sense
pointswithclass=pointswithclass(CorrespondingRange>20,:);

%discreatise into grid
class_ocgrid=[round(pointswithclass(:,1:3)/class_cubesize),pointswithclass(:,4)];
[level1]=GetImpLevInfo(class_ocgrid(:,1:3)*class_cubesize);
class_ocgrid=class_ocgrid(level1,:);

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
[level1]=GetImpLevInfo(workspace.ocgrid(:,1:3)*class_cubesize);
workspace.ocgrid=workspace.ocgrid(level1,:);
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


%we only want to hold the voxels which correspond with the current map
aabb = [-2, -2, -1; 2, 2, 2.4];
% aabb = [-inf, -inf, -inf; inf, inf, inf];
hMesh = robmap_h.Mesh(aabb);
f = hMesh.FaceData;
v = hMesh.VertexData;
[level1]=GetImpLevInfo(v);
v_decrete=unique(round(v(level1,:)/class_cubesize),'rows');
[vals,indexa,indexb]=intersect(v_decrete,workspace.ocgrid(:,1:3),'rows');
workspace.ocgrid=workspace.ocgrid(indexb,:);

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
