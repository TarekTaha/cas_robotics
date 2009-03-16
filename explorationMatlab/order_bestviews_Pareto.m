%% order_bestviews_Pareto
%
% *Description:*  Using the Pareto front to rank and select the optimal
% based upon the 4 functions: Geometric Info, C-space Info, Movement and
% Safety

%% Function Call
%
% *Inputs:* 
%
% _redo_nbv_vol_ (binary) wether or not to redo the nbv volume and pose checking
% for each on the bestviews
%
% *Returns:* NULL

function order_bestviews_Pareto()

%% Variables
global bestviews r densoobj workspace

dotweight=workspace.dotweight;

% obstacle_points=workspace.indexedobsticles;
obsticle_points=workspace.indexedobsticles(GetImpLevInfo(workspace.indexedobsticles),:);

% Determine the points in workspace
index=GetImpLevInfo(workspace.unknowncoords);
unknownpoints=workspace.unknowncoords(index,:);

numlinks = r.n;
Links = r.link; 
t = r.base;
qlimits=r.qlim; 
L = r.link;
for piece=1:numlinks
    linkvals(piece).val=[L{piece}.alpha L{piece}.A L{piece}.D L{piece}.offset];
end

load cart_config_correl_uniquebiglist.mat
    
%% Collect all weights so they can be sorted
%assumes all views are valid

for cur_view=1:size(bestviews,2)
    try 
        bestviews(cur_view).f_1=determineexpepctedgain(bestviews(cur_view).expectedaddinfo,dotweight);
        f_1(cur_view)=bestviews(cur_view).f_1;
        
        bestviews(cur_view).f_2=determineCpsace(bestviews(cur_view).expectedaddinfo,uniquebiglist,unknownpoints);
        f_2(cur_view)=bestviews(cur_view).f_2;
        
        bestviews(cur_view).f_3=sum(sum(abs(diff(bestviews(cur_view).all_steps))));
        f_3(cur_view)=bestviews(cur_view).f_3;
        
        bestviews(cur_view).f_4=determineAlgebraicDist(bestviews(cur_view).all_steps,obstacle_points,linkvals,densoobj,numlinks,t);
        f_4(cur_view)=bestviews(cur_view).f_4;
    catch
keyboard
    end      
end


%% Sort the remaining bestviews
[vals,index_f_1]=sort(f_1,'descend');
[vals,rank_f_1]=sort(index_f_1,'ascend');

[vals,index_f_2]=sort(f_2,'descend');
[vals,rank_f_2]=sort(index_f_2,'ascend');

[vals,index_f_3]=sort(f_3,'ascend');
[vals,rank_f_3]=sort(index_f_3,'ascend');

[vals,index_f_4]=sort(f_4,'descend');
[vals,rank_f_4]=sort(index_f_4,'ascend');

%add up all ranks
indexsum=rank_f_1+rank_f_2+rank_f_3+rank_f_4;
%find the best one which has the smallest overall rank
[vals,summedindexSorted]=sort(indexsum,'ascend');

tempbestviews=bestviews;
cur_rank=1;
for cur_view=summedindexSorted
    tempbestviews(cur_rank)=bestviews(cur_view);
    cur_rank=cur_rank+1;
end

tempbestviews=bestviews;


%% find the expected information gain
function H_1=determineexpepctedgain(expectedgain,dotweight)
[level1,level2,level3]=GetImpLevInfo(expectedgain);
H_1=log(2)*(dotweight(1)*size(level1,1)+dotweight(2)*size(level2,1)+dotweight(3)*size(level3,1));

%% The second information gaion - C-space
function H_2=determineCpsace(expectedgain,uniquebiglist,unknownpoints)
global workspace

all_interferance=sum(uniquebiglist(:,2));

%find the first level of info of the expectedgain
[level1]=GetImpLevInfo(expectedgain);

%find out which nodes will be seen by this view and look at their usage
[nothing, ia, ib]=intersect(expectedgain(level1,:),unknownpoints(uniquebiglist(:,1),:),'rows');

%prob is the interferance of this node divided by all interferances
%determine all probs at once
probs=uniquebiglist(:,2)/size(uniquebiglist,1);
%make a one sided uncertainty so max information is at prob=0.5
H_2=sum(-(probs/2).*log(probs/2));


% prob=sum(uniquebiglist(ib,2))/size(uniquebiglist,1);
% %make a one sided uncertainty so max information is at prob=0.5
% H_2=-(prob/2)*log(prob/2);


%% Find smallest algebraic Dist in a path 
function AlgebraicDist=determineAlgebraicDist(all_steps,obstacle_points,linkvals,densoobj,numlinks,robbase)

AlgebraicDist=inf;

for curr_step=1:size(all_steps,1)
    q_temp=all_steps(curr_step,:);
    tr=robbase;
    for i=1:numlinks; 
        tr = tr * linktransform_gp(linkvals(i).val,(q_temp(i)));
        if i>2 && i<7
            translated_points_1=obstacle_points(:,1)-tr(1,4);
            translated_points_2=obstacle_points(:,2)-tr(2,4);
            translated_points_3=obstacle_points(:,3)-tr(3,4); 

            %correct transform method *t(1:3,1:3)'
            translated_points_1_t=translated_points_1*tr(1,1)+translated_points_2*tr(1,2)+translated_points_3*tr(1,3);               
            translated_points_2_t=translated_points_1*tr(2,1)+translated_points_2*tr(2,2)+translated_points_3*tr(2,3);                      
            translated_points_3_t=translated_points_1*tr(3,1)+translated_points_2*tr(3,2)+translated_points_3*tr(3,3);                   

            [temp_AlgebraicDist]=min(((translated_points_1_t-densoobj(i+1).ellipse.center(1))/densoobj(i+1).ellipse.params(1)).^2+...
                                     ((translated_points_2_t-densoobj(i+1).ellipse.center(2))/densoobj(i+1).ellipse.params(2)).^2+...
                                     ((translated_points_3_t-densoobj(i+1).ellipse.center(3))/densoobj(i+1).ellipse.params(3)).^2);

             AlgebraicDist=min(temp_AlgebraicDist,AlgebraicDist);
             if AlgebraicDist<1
                 return; %since it isn't a valid path anyway
             end
        end
    end
end

