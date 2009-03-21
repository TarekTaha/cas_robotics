%% NBV_beta2
%
% *Description:*  Goes through all_views variable previously calculated and
% checks which poses are now safe, then goes through these views and
% removes point already calculated if their are new planes obstructing 

%% Function Call
%
% *Inputs:* NULL
%
% *Returns:* NULL

function NBV_beta2()

%% Variables
% clear the plots and global bestviews variable
clear global bestviews;

global workspace all_views bestviews scan r Q optimise;


%must be able to get to 0 and from current pose
[pathfound,all_steps_to0]=pathplanner_new([Q(1:3)+eps,zeros([1,r.n-3])],0,1,0,optimise.numofPPiterations,0);
if pathfound==0
  workspace.indexedobsticles=remove_self_scanning(workspace.indexedobsticles,Q);
  [pathfound,all_steps_to0]=pathplanner_new([Q(1:3)+eps,zeros([1,r.n-3])],0,1,0,optimise.numofPPiterations,0);
  if pathfound==0
    warning('You must be able to get back to zero from the current pose')
    keyboard
  end
end


%%%%%%%%%saving for the exhastive search
% try load Xsearchdata.mat;
%     Xsearchdata(end+1).workspace=workspace;
% catch;
%     Xsearchdata(1).workspace=workspace;
% end
% save('Xsearchdata.mat','Xsearchdata');
%%%%%%%%end saving file

%since we are no longer loading with exGUi we put it here to load if it
%hasn't already been/ or created if it dosen't exist
if isempty(all_views)
    try load all_views.mat
    catch
        display('Having to calculate all_views for exploration, this happens ones only');
        calc_all_views();
        load all_views.mat
    end    
end

n=r.n;
L = r.link;
for piece=1:n   
    linkvals(piece).val=[L{piece}.alpha L{piece}.A L{piece}.D L{piece}.offset];   
end

tic

indexed_knowncoords=round(setdiff(workspace.knowncoords(GetImpLevInfo(workspace.knowncoords),:),workspace.indexedobsticles,'rows')/workspace.inc_size);
%this makes the check for a collision quicker
obsticle_points=workspace.indexedobsticles(GetImpLevInfo(workspace.indexedobsticles),:);
all_possible=round(workspace.unknowncoords(workspace.lev1unknown   ,:)/workspace.inc_size);
[nothing,index]=setdiff(all_possible,[indexed_knowncoords;obsticle_points],'rows');
unknown_points=workspace.unknowncoords(workspace.lev1unknown(index),:);

all_known=sortrows([workspace.knowncoords;workspace.indexedobsticles]);

unknownweight=calunknownweight();

% find valid configs
% pos_validconfigs=find(all_views.result==-1);
pos_validconfigs=find(all_views.result==-1 | all_views.result==1);

%recheck the said to be valid configs
for cur_con=pos_validconfigs'
    [obstacle_result,unknown_result]=check_path_for_col(all_views.newQ(cur_con,:),obsticle_points,unknown_points,linkvals);   
    if ~obstacle_result
        all_views.result(cur_con)=0;
    elseif ~unknown_result %we are in unknown space
        all_views.result(cur_con)=-1;
    else
        all_views.result(cur_con)=1;
    end   
end

if size(scan.done_bestviews_orfailed,1)>0
    [nothing,nolongervalid]=intersect(all_views.newQ,scan.done_bestviews_orfailed,'rows');
    %set the result to 0 since we have already done it or it has failed somewhere
    all_views.result(nolongervalid)=0;
    scan.done_bestviews_orfailed=[inf,inf,inf,inf,inf,inf];
end

%delete all unneeded ones from memory
all_views=remove_all_views_result0(all_views);

[nothing,scanoriginOKindex]=intersect(all_views.scanorigin,workspace.knowncoords(GetImpLevInfo(workspace.knowncoords),:),'rows');
%update the valid config var

validconfigs=intersect(find(all_views.result==1),scanoriginOKindex);
if size(validconfigs,1)==1
  error('There are no valid configs possible in currently known space');
end

bestviews(size(validconfigs,1)).tr=zeros(4);
bestviews(size(validconfigs,1)).chosenview=zeros([1,3]);
bestviews(size(validconfigs,1)).scanorigin=zeros([1,3]);
bestviews(size(validconfigs,1)).Q=zeros([1,6]);
bestviews(size(validconfigs,1)).expectedaddinfo=[];
bestviews(size(validconfigs,1)).addinfo=0;
bestviews(size(validconfigs,1)).all_views_val=0;
    
BVcount=1;

for cur_con=validconfigs'
    points=all_views.expectedaddinfo(cur_con).vals;
    %nothing new found at this pose
    if size(points,1)==0
      bestviews(cur_con).tr=zeros(4);
      bestviews(cur_con).chosenview=zeros([1,3]);
      bestviews(cur_con).scanorigin=zeros([1,3]);
      bestviews(cur_con).Q=zeros([1,6]);
      bestviews(cur_con).expectedaddinfo=[];
      bestviews(cur_con).addinfo=0;
      bestviews(cur_con).all_views_val=0;
      continue;
    end

%% Remove points that are beyond osticle points
    %obstructing planes
    plane_homepnts=[workspace.indexedobsticles_home_point];
    %equations of planes
    plane_equ=[workspace.indexedobsticles_equ];
    %if any are the same then probably there is some obstruction by the planes
    plane_index=(1:size(plane_homepnts,1))';
    % remove points which are obscured by surfaces
        %scanorigin
        % o-------------
    %     \ Known
    %      \ 0\ obsticle
    %       \XX\ unknown since behind obsticle
    %        \XXX\

    % firstly remove all points that we either know are free or have an obstacle
    if size(all_known,1)<size(workspace.unknowncoords,1)
%         points=setdiff(points,all_known,'rows');
        
        %alternate setdiff method
        [sortedlist,listindex]=sortrows([points;all_known]);

        %since both lists are unique, we only want 1 point if that point is in both lists      
        uniquerow=[sortedlist(1:size(points,1)+size(all_known,1)-1,:)~=sortedlist(2:size(points,1)+size(all_known,1),:);...
                    true,true,true]; %last row is true by default since not compared to anything
        %get the ones where at least 1 row is different and the index is
        %out of the first set
        sortedlistIndex=(uniquerow(:,1)|uniquerow(:,2)|uniquerow(:,3))&...
                        listindex<size(points,1);
        %update the points list with only new (not known) points
        points=sortedlist(sortedlistIndex,:);        
%         size(points)
    else
        points=intersect(points,workspace.unknowncoords,'rows');
    end

    %nothing new found at this pose
    if size(points,1)==0
      bestviews(cur_con).tr=zeros(4);
      bestviews(cur_con).chosenview=zeros([1,3]);
      bestviews(cur_con).scanorigin=zeros([1,3]);
      bestviews(cur_con).Q=zeros([1,6]);
      bestviews(cur_con).expectedaddinfo=[];
      bestviews(cur_con).addinfo=0;
      bestviews(cur_con).all_views_val=0;
      continue;
    end
%% Go through each obstacle where the home point is within the scan
% $$ \begin{array}{l}
% \mbox{PlaneEq...} ax_p+by_p+cz_p+d=0\\
% \mbox{PlaneCenter...} P_c=(x_c,y_c,z_c)\\
% \mbox{PlaneRadius...} \mu\\
% \mbox{ScanOrigin...} P_s=(x_s,y_s,z_s)\\
% \mbox{PointsOnOpSide...} P_{o(j)}=(x_o,y_o,z_o)_{j=1...m}\\
% \end{array}$$
    % showld be the same as the surfaces made when we get obsticle points
    mew=workspace.mew; 
    current_scan_origin=all_views.scanorigin(cur_con,:);
    for i=plane_index'           
        %what side are the points on of this plane? 
        points_sign=(plane_equ(i,1)*points(:,1)+...
                     plane_equ(i,2)*points(:,2)+...
                     plane_equ(i,3)*points(:,3)+...
                     ones([size(points,1),1])*plane_equ(i,4))>0;

        %what side is the scan.origin on?
        scan_origin_sign=(plane_equ(i,1)*current_scan_origin(1)+...
                          plane_equ(i,2)*current_scan_origin(2)+...
                          plane_equ(i,3)*current_scan_origin(3)+...
                          plane_equ(i,4))>0;

        %All point on the same side are automatically valid        
        %Now look at point on the opisite side of the plane
        points_on_oposite_side=points(points_sign~=scan_origin_sign,:);       

%% If points on other side of this plane, ray trace, check if intersec<mew
% $$ \begin{array}{l}
% r=P_s-P_o \\
% t=\frac{ax_s+by_s+cz_s+d}{ar_x+br_y+xr_z}\\
% P_{intersection}=tr+P_s=(tr_x+x_s, tr_y+y_s, tr_z+z_s)=P_{i(k)}=(x_i,y_i,z_i)_{k=1...n}\\
% \forall(n), \sqrt{(x_i-x_c)^2+(y_i-y_c)^2+(z_i-z_c)^2}>\mu 
% \rightarrow \mbox{unobscured points}
% \end{array}$$
        if size(points_on_oposite_side,1)>0
            %we get the r variables for the parametric forms of a line between 2 points
            r_var=[current_scan_origin(1)-points_on_oposite_side(:,1),...
                   current_scan_origin(2)-points_on_oposite_side(:,2),...
                   current_scan_origin(3)-points_on_oposite_side(:,3)];               

            %find intersection point between surface and the scan line between scan origin and point
            bottomof_t_var=plane_equ(i,1)*r_var(:,1)+...
                           plane_equ(i,2)*r_var(:,2)+...
                           plane_equ(i,3)*r_var(:,3);
            %make sure it is not 0 otherwise change it so it is simply a very small number (epsilon) so we can keep the size of matrixes
            if ~isempty(find(bottomof_t_var==0, 1)); bottomof_t_var(bottomof_t_var==0)=eps; end                                                                               
            t_var=( plane_equ(i,1)*current_scan_origin(1)+...
                    plane_equ(i,2)*current_scan_origin(2)+...
                    plane_equ(i,3)*current_scan_origin(3)+...
                    plane_equ(i,4)...
                   )./ bottomof_t_var;                 

            % Get the intersection points
            intersectionPNTs=[t_var.*-r_var(:,1)+current_scan_origin(1),...
                              t_var.*-r_var(:,2)+current_scan_origin(2),...
                              t_var.*-r_var(:,3)+current_scan_origin(3)];

            %find the points which are either on the same side of the plane as the scanning point 
            %or they are on the other side and are less than mew fmro the intersection point       
            points=[points(points_sign==scan_origin_sign,:);...
                    points_on_oposite_side((sqrt((intersectionPNTs(:,1)-plane_homepnts(i,1)).^2+...
                                                 (intersectionPNTs(:,2)-plane_homepnts(i,2)).^2+...
                                                 (intersectionPNTs(:,3)-plane_homepnts(i,3)).^2)>mew),:)];
           %it is possible that all points have been removed so we can break
           %out and say no points are give (if surface is right in front)
            if size(points,1)==0
                break
            end
        end
    end

%% update the expected info and update best views
    all_views.expectedaddinfo(cur_con).vals=points;

    bestviews(BVcount).tr=squeeze(all_views.tr(cur_con,:,:));
    bestviews(BVcount).chosenview=sum(bestviews(BVcount).tr(1:3,1:3));
    bestviews(BVcount).scanorigin=bestviews(BVcount).tr(1:3,4)*workspace.inc_size;
    bestviews(BVcount).Q=all_views.newQ(cur_con,:);                                
    bestviews(BVcount).expectedaddinfo=all_views.expectedaddinfo(cur_con).vals;
    bestviews(BVcount).addinfo=getweighted_addinfo(bestviews(BVcount).expectedaddinfo)/unknownweight;
    bestviews(BVcount).all_views_val=cur_con;

    BVcount=BVcount+1;
end

%% Remove bestviews where there is insignificant gain
display('Removing invalid bestviews')
tic
invalid_views=[];


for cur_view=1:size(bestviews,2)  
  if isempty(bestviews(cur_view).expectedaddinfo)  
    invalid_views=[invalid_views;cur_view];
  end
end
if size(invalid_views,1)>0
  current_count=1;
  for cur_view=1:size(bestviews,2)
    if isempty(find(invalid_views==cur_view,1))
      temp_bestviews(current_count)=bestviews(cur_view);
      current_count=current_count+1;
    end
  end
  try 
    bestviews=temp_bestviews;
  catch bestviews=[];
  end
end

toc

%% Check if any bestviews were found, if so order them
if isempty(bestviews)
    error('There were no bestviews found, probably because there are no known points to go too');
end

%order the best views
%order_bestviews()

%% NEW RRT PLANNER FROM STEVE
% global occHandle
% MotionPlanner = actxserver('EyeInHand.DensoMotionPlan');
% % MotionPlanner.OccupancyData = occHandle;
% MotionPlanner.SurfaceData=robmap_h;
% for current_view=1:size(bestviews,2)
%   MotionPlanner.InitialConfiguration=rad2deg(Q);
%   MotionPlanner.TargetConfiguration=rad2deg(bestviews(current_view).Q);
% %   MotionPlanner.CanStart
%   MotionPlanner.Start; 
%   tic
%   MotionPlanner.WaitUntilCompleted(1.0);
%   toc
%   if strcmpi(MotionPlanner.Status, 'Started')
%     MotionPlanner.Stop;
%     display(['Stopping',num2str(current_view)]);
%     pause(1);
%   end
% 
%   bestviews(current_view).all_steps=deg2rad(MotionPlanner.ConfigurationPath);
% end

%% Do new wavefront based planner
newQ=zeros([size(bestviews,2),6]);
for current_view=1:size(bestviews,2)
  
  try [pathfound,bestviews(current_view).all_steps_from0]=pathplanner_new([bestviews(current_view).Q(1:3)+eps,bestviews(current_view).Q(4:6)],0,1,0,optimise.numofPPiterations,0,[bestviews(current_view).Q(1:3),zeros([1,r.n-3])]);
  catch
    pathfound=0;
    bestviews(current_view).all_steps_from0=[];
  end
  
  if pathfound
    newQ(current_view,:)=[bestviews(current_view).Q(1:3),zeros([1,r.n-3])];
  else   %no path from newQ(1:3),0,0,0 to newQ found so going to try anyway with no zeros
    newQ(current_view,:)=bestviews(current_view).Q;
    %so clear the all_steps_from0 since it is crap values anyway
    bestviews(current_view).all_steps_from0=[];
  end
end


pathval=pathplanner_water(newQ,0,0,1,[Q(1:3)+eps,zeros([1,r.n-3])]);


valid_count=0;
tempbestviews=[];
encroachIntoUnknown=[];
correspondingindex_used=[];

for current_view=1:size(pathval,2)
  
  
  if pathval(current_view).result 
    if ~isempty(find(diff(abs(pathval(current_view).all_steps(:,1:3)))>20*pi/180,1))
      %then its not valid
      display('path with greater than 20 deg leap at end detected so ignoring it');
      continue;          
    end
    
    if pathval(current_view).unknown_points_result
      encroachIntoUnknown=[encroachIntoUnknown,current_view];
    end
    valid_count=valid_count+1;
    %find the corresponding best view to this pose
    correspondingindex=find(newQ(:,1)-pathval(current_view).all_steps(end,1)==0 &...
                            newQ(:,2)-pathval(current_view).all_steps(end,2)==0 & ...
                            newQ(:,3)-pathval(current_view).all_steps(end,3)==0 & ...
                            newQ(:,4)-pathval(current_view).all_steps(end,4)==0 & ...
                            newQ(:,5)-pathval(current_view).all_steps(end,5)==0 & ...
                            newQ(:,6)-pathval(current_view).all_steps(end,6)==0);
                          
    if isempty(correspondingindex)
      error('Couldnt find the corresponding best view');
    elseif size(correspondingindex,1)>1
      %find the first one that hasn't been seen before
      temp=setdiff(correspondingindex,correspondingindex_used);
      correspondingindex=temp(1);      
    end
    %store the ones already seen
    correspondingindex_used=[correspondingindex_used;correspondingindex];
    
    %find the correct path
    bestviews(correspondingindex).valid=pathval(current_view).result;
    
    %if we have some views to add to the end and the first step is equal to
      %the last step of pathval(current_view).all_steps (within eps) the get
    %rid of it
    if ~isempty(bestviews(current_view).all_steps_from0)
      if isempty(find(pathval(current_view).all_steps(end,:)'-bestviews(current_view).all_steps_from0(1,:)'>eps,1))
        try bestviews(current_view).all_steps_from0=bestviews(current_view).all_steps_from0(2:end,:);
        catch 
          display('NBV_beta2 :: couldnt take off the first step of bestview 0 to actual newQ');
        end
      end
    end
    %add steps at start to get to 0's and then at end to get from 0's to newQ
    bestviews(correspondingindex).all_steps=[all_steps_to0;
                                             pathval(current_view).all_steps;
                                             bestviews(current_view).all_steps_from0];
                                           
                                             
      

    %put old bestview data into the temp bestview (need to assign to whole
    %of tempbestviews if it is the first structure assignment)
    if valid_count==1; 
      tempbestviews=bestviews(correspondingindex);
    else
      tempbestviews(valid_count)=bestviews(correspondingindex);
    end
  end  
end
%if any go into unknown list them but don't ignore
if size(encroachIntoUnknown,1)>0
    display(['Some bestview may encroch upon some unknown space in the inbetween links of its path, but still including them']);
end

if valid_count>0
  bestviews=tempbestviews;
  display(['There were ',num2str(valid_count),' bestviews found with paths'])
  
  try order_bestviews_Pareto()
  catch
    warning('NOT ORDERING properly, using old method');order_bestviews();
  end
  
else
  error('There were no bestviews found after path planning');
end
toc
