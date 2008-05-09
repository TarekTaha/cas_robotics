%% Pose Selection To Classify Unknown
% Description: This function is used to identiy areas which have unknown
% classification and attempt to look at them in such a way that we can do a
% classification
function poseclassunknown()
error('This dosent work, it is for reference only');

%% Setup and Variables
close all
global r Q scan workspace

%% Load data or classify a scan
doload_not_scan=false;
keyboard
if doload_not_scan
    load GavData
else
    NOhandleOPTIONS.useRealRobot=false;
    NOhandleOPTIONS.show_robot=true;
    NOhandleOPTIONS.animate_move=true;
    NOhandleOPTIONS.remv_unkn_in_mv=false;    
%     movetonewQ(0,newQ,all_steps,NOhandleOPTIONS);
    % or get scan data from exGUI
    if isstruct(scan)
       try 
           ClassifiedData=scan.ClassificationData;
            PointData=scan.PointData;
       catch
           error('Scan variable does not have the required fields');
       end
    else
        error('For now you need to run with exGUI');
    end
end

%%

% the classifier reterns a classification of this number is unknown
unknownclass=8;
%size of the surfaces which is 1/2 of 20' scan at ideal distance 0.5*tan(20*pi/180)
mew=0.1;
% This is how many sets of data we will classify
numofintplanes=30;
%how many solutions and failures
solsfound=0;
noposefound=0;
%set start pose
Q=[0,0,0,0,0,0];

%optimisataion variables
global classunkn_optimise
classunkn_optimise.minSurfToEF=0.3;
classunkn_optimise.maxSurfToEF=0.8;
classunkn_optimise.distAwayfromTarget=0.6;
classunkn_optimise.maxAngle=20*pi/180;

classunkn_optimise.iLimit=100;
classunkn_optimise.stol=1e-4;

%% put classification data in one big matrix
pointswithclass=zeros([size(PointData,1)*size(PointData,2),4]);
for i=1:size(PointData,1);
    pointswithclass((i-1)*size(PointData,2)+1:i*size(PointData,2),:)=[squeeze(PointData(i,:,:)),ClassifiedData(i,:)'];
end;

%find unknown points
unknownindex=find(pointswithclass(:,4)==unknownclass);

%% plot and make surfaces out of these
plot3(pointswithclass(unknownindex,1),pointswithclass(unknownindex,2),pointswithclass(unknownindex,3),'r.')

surface_making_simple(pointswithclass(unknownindex,1:3),mew)
global plane

% plot_planes(plane,mew);
axis([-1,1,-1,1,0,1.5]);
% axis equal
camlight

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
for i=1:numofintplanes

% plot plane and points 
    try for j=1:length(planeplotHa);  delete(planeplotHa(j));end; end
    planeplotHa=plot_planes(plane(index(i)),mew);    

    hold on;
    planeplotHa(2)=plot3(plane(index(i)).home_point(1),plane(index(i)).home_point(2),plane(index(i)).home_point(3),'b*');
    planeplotHa(3)=plot3([plane(index(i)).home_point(1),plane(index(i)).home_point(1)+plane(index(i)).normal_by_eigenval(1)/10],...
                         [plane(index(i)).home_point(2),plane(index(i)).home_point(2)+plane(index(i)).normal_by_eigenval(2)/10],...
                         [plane(index(i)).home_point(3),plane(index(i)).home_point(3)+plane(index(i)).normal_by_eigenval(3)/10],'b');    
                     
    
     
    pt=plane(index(i)).home_point;
    plane_equ=plane(index(i)).equ;
    
    try [qt,solutionvalid,dist_val,targetdist]=classunk_posesel(pt, plane_equ, Q);
        if solutionvalid
            solsfound=solsfound+1;
            display(['Solution found = ',num2str(solsfound)]);
            title(['Solution found = ',num2str(solsfound)]);
            plotdenso(r,qt);drawnow;
            Q=qt;
        else
            noposefound=noposefound+1;            
            display(['no pose found = ', num2str(noposefound)]);
            title(['no pose found = ', num2str(noposefound)]);
        end
    catch   
        noposefound=noposefound+1;
        display('Some error');
        title(['Some error:no pose found = ', num2str(noposefound)]);
    end
end





