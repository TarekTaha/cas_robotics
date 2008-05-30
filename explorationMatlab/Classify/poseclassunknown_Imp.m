%% Pose Selection To Classify Unknown - Implementation Ver
% Description: This function is used to identiy areas which have unknown
% classification and attempt to look at them in such a way that we can do a
% classification
function poseclassunknown_Imp(numofintplanes)

%% Setup and Variables
% close all
global r Q PointData RangeData IntensityData workspace classPlanePlotHa

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
class_cubesize=workspace.class_cubesize;

% minclassifications=workspace.minclassifications;
% classfierthreshhold=workspace.classfierthreshhold;

% the classifier reterns a classification of this number is unknown
% unknownclass=8;
%size of the surfaces which is 1/2 of 20' scan at ideal distance 0.5*tan(20*pi/180)
mew=0.1;
% This is how many sets of data we will classify
% numofintplanes=20;

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
try UNclassifiedvoxels=update_ocstatus(ClassifiedData);
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

%% Go through the number of planes we are interested in
indextoblast=0;

while solsfound<numofintplanes
    indextoblast=indextoblast+1;
% plot plane and points 
    for j=1:length(classPlanePlotHa);  try delete(classPlanePlotHa(j));end; end
    clear global classPlanePlotHa;
    classPlanePlotHa=plot_planes(plane(index(indextoblast)),mew);    

    hold on;
    classPlanePlotHa(2)=plot3(plane(index(indextoblast)).home_point(1),plane(index(indextoblast)).home_point(2),plane(index(indextoblast)).home_point(3),'b*');
    classPlanePlotHa(3)=plot3([plane(index(indextoblast)).home_point(1),plane(index(indextoblast)).home_point(1)+plane(index(indextoblast)).normal_by_eigenval(1)/10],...
                         [plane(index(indextoblast)).home_point(2),plane(index(indextoblast)).home_point(2)+plane(index(indextoblast)).normal_by_eigenval(2)/10],...
                         [plane(index(indextoblast)).home_point(3),plane(index(indextoblast)).home_point(3)+plane(index(indextoblast)).normal_by_eigenval(3)/10],'b');    
                     
         
    pt=plane(index(indextoblast)).home_point;
    plane_equ=plane(index(indextoblast)).equ;
    
    try [newQ,solutionvalid]=classunk_posesel(pt, plane_equ, Q);
        if solutionvalid            
            %make sure vector is correct
            newQ=newQ(:)';
                       
            %move to the next place if possible, else continue with the
            %next plane
            if movetonewQ(0,rad2deg(newQ),[],NOhandleOPTIONS);            
                solsfound=solsfound+1;
                display(['Solution found and found a path successfully there = ',num2str(solsfound)]);
            else
                continue;
            end
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
            
            UNclassifiedvoxels=update_ocstatus(ClassifiedData);
            uiwait(msgbox('press OK button to continue'));
            
            
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
    end
end
