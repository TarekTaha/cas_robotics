%% Determine poses and paths for classifier
% pass me the index of valid planes and the ,UNclassifiedvoxels
% I will return a list of planeSet, poses for them and paths to get there

function [planeSet,pose,pathval]=determinePathsNposes(index)

global classunkn_optimise optimise plane Q

%% VARIABLE

% how many planes we are after in one go
maxindexsize=20;
maxangleJ4to6=optimise.maxangleJ4to6;

%make a planeset out of the desirable planes
if size(index,1)>maxindexsize
    planeSet=plane(index(1:maxindexsize));
else
    planeSet=plane(index);
end



%%
%change optimisation parameters to be for viewing
optimise.maxDeflectionError=classunkn_optimise.maxAngle;
optimise.minDeflectionError=0;
optimise.mintargetdis=classunkn_optimise.minSurfToEF;
optimise.maxtargetdis=classunkn_optimise.maxSurfToEF;
optimise.minAccepDis=classunkn_optimise.distAwayfromTarget;           
pose=PoseSel4planesearch(planeSet,false,Q);
%setup optimisation correctly again
setupoptimisation() 

%% Extract Next Steps

newQ=[];
validposes=[];

for curr_pose=1:size(pose,2)
    if pose(curr_pose).validPose
        validposes=[validposes;curr_pose];
        newQ=[newQ;pose(curr_pose).Q];
    end
end
%only keep the ones for which we have a valid pose for
planeSet=planeSet(validposes);
pose=pose(validposes);

%% move last joints to all zeros them the first 3 then the last 3 back to

if ~isempty(find(Q(4:end)>maxangleJ4to6,1))
    last4to0newQ=[Q(1)*ones([size(newQ,1),1]),Q(2)*ones([size(newQ,1),1]),Q(3)*ones([size(newQ,1),1]),zeros([size(newQ,1),size(newQ,2)-3])];
    pathvaltemp1=pathplanner_water(last4to0newQ,false);
else %then make each one a movement from Q to Q with 4:end =0 (result of 1)
    for curr_pose=1:size(pose,2)
        pathvaltemp1(curr_pose).result=1;
        pathvaltemp1(curr_pose).all_steps=[Q;Q(1:3),zeros([1,size(newQ,2)-3])];
    end
end

% set 4:end of currQ to 0, 
currQ=[Q(1:3),zeros([1,size(newQ,2)-3])];
%Change NewQ: newQ with 4:end as 0
newQlast4allzero=[newQ(:,1:3),zeros([size(newQ,1),size(newQ,2)-3])];
%path plan to all the changed newQs (some may be invalid since they weren't checked
[pathvaltemp2,validposes]=pathplanner_water(newQlast4allzero,false,currQ);

%resize mats based upon validposes
pathvaltemp1=pathvaltemp1(validposes);
newQ=newQ(validposes,:);
planeSet=planeSet(validposes);
pose=pose(validposes);

validposes=[];
%now go from end of newQ with 4:end as zeros to the final newQ pose
%determined
for curr_pose=1:size(pathvaltemp1,2)
    if isempty(find(abs(pathvaltemp2(curr_pose).all_steps(end,4:end)-newQ(curr_pose,4:end))>maxangleJ4to6,1)) 
        pathvaltemp3(1).all_steps=newQ;
        pathvaltemp3(1).result=1;
    else        
        currQ=pathvaltemp2(curr_pose).all_steps(end,:);
        
        try [pathvaltemp3(1).result,pathvaltemp3(1).all_steps] = pathplanner_new(newQ(curr_pose,:),0,1,0,0,0,currQ);
        catch
            keyboard
        end
    end
    
    %now combine them
    pathval(curr_pose).result=pathvaltemp1(curr_pose).result==1 & pathvaltemp2(curr_pose).result==1 & pathvaltemp3(1).result==1;
    pathval(curr_pose).all_steps=[pathvaltemp1(curr_pose).all_steps;...
                                  pathvaltemp2(curr_pose).all_steps;...
                                  pathvaltemp3(1).all_steps];
    pathval(curr_pose).newQ=newQ(curr_pose,:);
    if pathval(curr_pose).result
        validposes=[validposes;curr_pose];
    end
end

%% Resize matrices
planeSet=planeSet(validposes);
pose=pose(validposes);
pathval=pathval(validposes);