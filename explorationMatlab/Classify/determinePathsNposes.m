%% Determine poses and paths for classifier
% pass me the index of valid planes and the ,UNclassifiedvoxels
% I will return a list of planeSet, poses for them and paths to get there

function [planeSet,pose,pathval]=determinePathsNposes(index,maxindexsize)

global classunkn_optimise optimise plane Q

%% VARIABLE

% how many planes we are after in one go
if nargin<2
  maxindexsize=20;
end
maxangleJ4to6=optimise.maxangleJ4to6;

%make a planeset out of the desirable planes
if size(index,1)>maxindexsize
    planeSet=plane(index(1:maxindexsize));
else
    planeSet=plane(index);
end

%%For testing purposes only, can delete otherwise
display('In determinePathsNposes, since doing testing we are setting useMyMethodOnly = true');useMyMethodOnly = true;

%%
%change optimisation parameters to be for viewing
optimise.maxDeflectionError=classunkn_optimise.maxAngle;
optimise.minDeflectionError=0;
optimise.mintargetdis=classunkn_optimise.minSurfToEF;
optimise.maxtargetdis=classunkn_optimise.maxSurfToEF;
optimise.minAccepDis=classunkn_optimise.distAwayfromTarget;           
pose=PoseSel4planesearch(planeSet,false,Q,useMyMethodOnly);
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


%% just use water pathplanner
%you must have last 4 joints at zero for this to work
pathval=pathplanner_water([newQ(:,1:3),zeros([size(newQ,1),3])]);
%set all invalid uless they proove otherwise
validposes=false([1,size(pathval,2)]);
correspondingindex=[];
for curr_pose=1:size(pathval,2)
  try correspondingindex_temp=find(newQ(:,1)-pathval(curr_pose).newQ(1)==0 &newQ(:,2)-pathval(curr_pose).newQ(2)==0 &newQ(:,3)-pathval(curr_pose).newQ(3)==0);
    if isempty(correspondingindex) || size(correspondingindex_temp,1)==1
      correspondingindex(curr_pose)=correspondingindex_temp(1);
    else
      temp_set=setdiff(correspondingindex_temp,correspondingindex);
      correspondingindex(curr_pose)=temp_set(1);
    end
  catch
    keyboard
  end
  if pathval(curr_pose).result==1
    validposes(curr_pose)=true;
  end
end
newQ=newQ(correspondingindex,:);
planeSet=planeSet(correspondingindex);
pose=pose(correspondingindex);

display(['Before moving last 4 joints Current valid size is ',num2str(size(find(validposes==1),2)), ' poses with paths'])
for curr_pose=1:size(newQ,1)
  if validposes(curr_pose) %otherwise don't bother with it
    [pathfound,all_steps]=pathplanner_new(newQ(curr_pose,:),1,0,optimise.numofPPiterations,0,[newQ(curr_pose,1:3),0,0,0]);
    if pathfound==1
      validposes(curr_pose)=true;
      pathval(curr_pose).all_steps=[pathval(curr_pose).all_steps;all_steps];
      pathval(curr_pose).newQ=newQ(curr_pose,:);  
    else
      validposes(curr_pose)=false;
    end
  end
end  

%% Resize matrices
planeSet=planeSet(validposes);
pose=pose(validposes);
if size(validposes,1)>0
  pathval=pathval(validposes);
  display(['determinePathsNposes:: found valid ',num2str(size(find(validposes==1),2)), ' poses with paths'])
else
  pathval=[];
end
  