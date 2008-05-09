%% Pose Selection To Classify Unknown - Implementation Ver
% Description: This function is used to identiy areas which have unknown
% classification and attempt to look at them in such a way that we can do a
% classification
function poseclassunknown_Imp()

%% Setup and Variables
% close all
global r Q PointData RangeData IntensityData workspace planeplotHa

% figure(1)
% plot_planes(plane,mew);
% axis([-1,1,-1,1,0,1.5]);
% axis equal
% camlight

%% Load data or classify a scan

NOhandleOPTIONS.useRealRobot=true;
NOhandleOPTIONS.show_robot=true;
NOhandleOPTIONS.animate_move=true;
NOhandleOPTIONS.remv_unkn_in_mv=false;    

%take one scan from an initial pose
% newQ=[0,-88*pi/180,140*pi/180,0,0*pi/180,0];
%move to the next place
% movetonewQ(0,rad2deg(newQ),[],NOhandleOPTIONS);
% Q=newQ;

% % and do a scan to classify
% if NOhandleOPTIONS.useRealRobot
%     try use_real_robot_SCAN(90); catch; display('Couldnt scan');end
% end
%             

            

%% More Vars
class_cubesize=workspace.inc_size/4;

%minimum number of classifications in voxel to make it valid
minclassifications=35;
% minimmum majority classification to make it useful
classfierthreshhold=1.5;

% the classifier reterns a classification of this number is unknown
% unknownclass=8;
%size of the surfaces which is 1/2 of 20' scan at ideal distance 0.5*tan(20*pi/180)
mew=0.1;
% This is how many sets of data we will classify
numofintplanes=20;
%how many solutions and failures
solsfound=0;
noposefound=0;



%optimisataion variables
global classunkn_optimise
classunkn_optimise.minSurfToEF=0.25;
classunkn_optimise.maxSurfToEF=0.8;
classunkn_optimise.distAwayfromTarget=0.47;
classunkn_optimise.maxAngle=42*pi/180;

classunkn_optimise.iLimit=500;
classunkn_optimise.stol=1e-6;

%% Classify
try [ClassifiedData] = Block_Classifier(PointData, IntensityData,RangeData); catch; display('Couldnt classify');end
% load tempdata.mat

%% put classification data in one big matrix
% display('TEMP LOADING DATA');load GavData;ClassifiedData=ClassifiedDatawEdges;
try UNclassifiedvoxels=update_ocstatus(class_cubesize,ClassifiedData,minclassifications,classfierthreshhold);
catch; keyboard;
end


%% plot and make surfaces out of these
% plot3(pointswithclass(unknownindex,1),pointswithclass(unknownindex,2),pointswithclass(unknownindex,3),'r.')

surface_making_simple(workspace.ocgrid(UNclassifiedvoxels,:)*class_cubesize,mew)
global plane



%% Find the planes covering most unknown points
% these are desirable places to look
sizemat=zeros([length(plane),1]);
for i=1:length(plane)
    sizemat(i)=size(plane(i).points,1);
end

% order these
[nothing,index]=sort(sizemat,'descend');
%make sure we have at max, the specified numofintplanes
if length(index)<numofintplanes
    numofintplanes=index;
end

%% Go through the number of planese we are interested in
indextoblast=0;
for i=1:numofintplanes
    indextoblast=indextoblast+1;
% plot plane and points 
    for j=1:length(planeplotHa);  try delete(planeplotHa(j));end; end
    clear planeplotHa;
    planeplotHa=plot_planes(plane(index(indextoblast)),mew);    

    hold on;
    planeplotHa(2)=plot3(plane(index(indextoblast)).home_point(1),plane(index(indextoblast)).home_point(2),plane(index(indextoblast)).home_point(3),'b*');
    planeplotHa(3)=plot3([plane(index(indextoblast)).home_point(1),plane(index(indextoblast)).home_point(1)+plane(index(indextoblast)).normal_by_eigenval(1)/10],...
                         [plane(index(indextoblast)).home_point(2),plane(index(indextoblast)).home_point(2)+plane(index(indextoblast)).normal_by_eigenval(2)/10],...
                         [plane(index(indextoblast)).home_point(3),plane(index(indextoblast)).home_point(3)+plane(index(indextoblast)).normal_by_eigenval(3)/10],'b');    
                     
    
     
    pt=plane(index(indextoblast)).home_point;
    plane_equ=plane(index(indextoblast)).equ;
    
    try [newQ,solutionvalid,dist_val,targetdist]=classunk_posesel(pt, plane_equ, Q);
        if solutionvalid
            solsfound=solsfound+1;
            display(['Solution found = ',num2str(solsfound)]);
%             title(['Solution found = ',num2str(solsfound)]);
            %make sure vector is correct
            newQ=newQ(:)';
                       
            %move to the next place
            movetonewQ(0,rad2deg(newQ),[],NOhandleOPTIONS);
            % and do a scan to classify
            
            if NOhandleOPTIONS.useRealRobot
                %try and move through a complete scan so we have to make newQ standard
                if newQ(5)>-45*pi/180 
                    if newQ(5)<60*pi/180
                        newQ=[newQ(1:4),newQ(5)+30*pi/180,0];
                    else
                        newQ=[newQ(1:4),90*pi/180,0];
                    end
                else
                    newQ=[newQ(1:4),-45*pi/180,0];
                end
                
                try movetonewQ(0,rad2deg(newQ),[],NOhandleOPTIONS);catch; error('Could move to correct position');end
                try use_real_robot_SCAN(-60); organise_data();catch; error('Could scan anything');end
                use_real_robot_GETJs();
                % and do a scan to classify           
                global PointData IntensityData RangeData
                try [ClassifiedData] = Block_Classifier(PointData, IntensityData, RangeData); catch; error('Couldnt classify');end
            end
            
            UNclassifiedvoxels=update_ocstatus(class_cubesize,ClassifiedData,minclassifications,classfierthreshhold);
            
            surface_making_simple(workspace.ocgrid(UNclassifiedvoxels,:)*class_cubesize,mew)
            global plane
            if size(plane,2)<1
                display('There are no more points to look at - returning');
                return
            end
            %% Find the planes covering most unknown pointsthese are desirable places to look
            sizemat=zeros([length(plane),1]);
            for j=1:length(plane)
                sizemat(j)=size(plane(j).points,1);
            end

            % order these
            [nothing,index]=sort(sizemat,'descend');
            indextoblast=0;
        else
            noposefound=noposefound+1;            
            display(['no pose found = ', num2str(noposefound)]);
        end
    catch   
        noposefound=noposefound+1;
        display('Some error');
%         title(['Some error:no pose found = ', num2str(noposefound)]);
    end
end


%function for updating the voxels status
function UNclassifiedvoxels=update_ocstatus(class_cubesize,ClassifiedData,minclassifications,classfierthreshhold)

global workspace PointData planeplotHa RangeData
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
[level1,level2]=GetImpLevInfo(class_ocgrid(:,1:3)*class_cubesize);
class_ocgrid=class_ocgrid(level2,:);

hold on;
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
[level1,level2]=GetImpLevInfo(workspace.ocgrid(:,1:3)*class_cubesize);
workspace.ocgrid=workspace.ocgrid(level2,:);

for i=1:size(workspace.ocgrid,1)
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


% figure(2)
sumofclass=workspace.ocgrid(:,4)+workspace.ocgrid(:,5);
warning('off','MATLAB:divideByZero')
try classifiedvoxels=find(sumofclass>=minclassifications &...
                     (workspace.ocgrid(:,4)./workspace.ocgrid(:,5)>classfierthreshhold |...
                      workspace.ocgrid(:,5)./workspace.ocgrid(:,4)>classfierthreshhold));end
try UNclassifiedvoxels=find(sumofclass<minclassifications | ...
                     (workspace.ocgrid(:,4)./workspace.ocgrid(:,5)<=classfierthreshhold &...
                      workspace.ocgrid(:,5)./workspace.ocgrid(:,4)<=classfierthreshhold));end
warning('on','MATLAB:divideByZero')
hold on;
try planeplotHa(end+1)=plot3(workspace.ocgrid(UNclassifiedvoxels,1)*class_cubesize,...
      workspace.ocgrid(UNclassifiedvoxels,2)*class_cubesize,...
      workspace.ocgrid(UNclassifiedvoxels,3)*class_cubesize,'y','marker','.','markersize',0.5,'linestyle','none');end
%plot metal and wood voxels

metalvoxels=workspace.ocgrid(classifiedvoxels(workspace.ocgrid(classifiedvoxels,4)>workspace.ocgrid(classifiedvoxels,5)),1:3);
if size(metalvoxels,1)>0
    try planeplotHa(end+1)=plot3(metalvoxels(:,1)*class_cubesize,metalvoxels(:,2)*class_cubesize,metalvoxels(:,3)*class_cubesize,'r.');end
end

woodvoxels=workspace.ocgrid(classifiedvoxels(workspace.ocgrid(classifiedvoxels,4)<workspace.ocgrid(classifiedvoxels,5)),1:3);
if size(woodvoxels,1)>0
    try planeplotHa(end+1)=plot3(woodvoxels(:,1)*class_cubesize,woodvoxels(:,2)*class_cubesize,woodvoxels(:,3)*class_cubesize,'b.');end
end

drawnow

