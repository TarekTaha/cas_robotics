%% calc_all_views
%
% *Description:*  This function goes through all the poses discretely and
% looks to see which ones give most innformation saves in a file that could
% be used by NBV_beta

%% Function Call
%
% *Inputs:* NULL
%
% *Returns:* NULL

function calc_all_views()

%% Variables
% clear the plots and global bestviews variable
clear global bestviews;

%this starts the timer
tic

global Q r workspace robot_maxreach;

%tempory make global Q something else
actualQ=Q;
Q=robot_maxreach.default_Q(end,:);

%actually will go through first step and then this many more 
numNBVanglesteps=6;
qlimit=r.qlim;
%taking 10 deg off either side so we don't use the max range for these joints
qlimit(1:3,1)=qlimit(1:3,1)+10*pi/180;
qlimit(1:3,2)=qlimit(1:3,2)-10*pi/180;


indexed_knowncoords=round(setdiff(workspace.knowncoords(GetImpLevInfo(workspace.knowncoords),:),workspace.indexedobsticles,'rows')/workspace.inc_size);

%this makes the check for a collision quicker
obsticle_points=workspace.indexedobsticles(GetImpLevInfo(workspace.indexedobsticles),:);

%%STARTADDED
%Additional (unknown info)
all_possible=round(   workspace.unknowncoords(workspace.lev1unknown   ,:)/workspace.inc_size);
[nothing,index]=setdiff(all_possible,[indexed_knowncoords;obsticle_points],'rows');
unknown_points=workspace.unknowncoords(workspace.lev1unknown(index),:);
%%ENDADDED

unknownweight=calcweight(0.5);

q1=qlimit(1,1):(qlimit(1,2)-qlimit(1,1))/numNBVanglesteps:qlimit(1,2);
q2=qlimit(2,1):(qlimit(2,2)-qlimit(2,1))/numNBVanglesteps:qlimit(2,2);
q3=qlimit(3,1):(qlimit(3,2)-qlimit(3,1))/numNBVanglesteps:qlimit(3,2);
q4=[-135,-45,135,45]*pi/180;

total_size=size(q1,2)*size(q2,2)*size(q3,2)*size(q4,2)*2;

all_views.result=zeros([total_size,1]);
all_views.newQ=zeros([total_size,6]);
all_views.tr=zeros([total_size,4,4]);
all_views.scanorigin=zeros([total_size,3]);
all_views.path(total_size).valid=0;
all_views.path(total_size).path=zeros([2,6]);

count=1;
%% Go through the joints at discrete positions
for J1=q1
    for J2=q2
        for J3=q3
            %Check collision which also checks soft mostion limit which we can't exceed  
            [obstacle_result,unknown_result]=check_path_for_col([J1,J2,J3,0,0,0],obsticle_points,unknown_points);
            if ~obstacle_result %leave it at 0 then get out of this IF                                   
                %all_views.result(count:count+7)=0*ones([8,1]);
            elseif ~unknown_result %we are in unknown space
                all_views.result(count:count+7)=-1*ones([8,1]);
            else %no problem
                all_views.result(count:count+7)=1*ones([8,1]);
            end

            %predefinedJ4's since we don't really need the whole range of movement
            for J4=q4
                %Sets up the limits (possible poses) of J5
                %depending on J3
                if J3>150*pi/180;  J5s_to_go_through=[-60,30]*pi/180; 
                else J5s_to_go_through=[-45,45]*pi/180; end                
                for J5=J5s_to_go_through                        
                    all_views.newQ(count,:)=[J1,J2,J3,J4,J5,0];
                    %if we haven't already done this pose before
                    all_views.tr(count,:,:)=fkine(r,all_views.newQ(count,:));                                    
                    all_views.scanorigin(count,:)=round(all_views.tr(count,1:3,4)/workspace.inc_size)*workspace.inc_size;
                    all_views.expectedaddinfo(count).vals=nbv_volume(squeeze(all_views.tr(count,:,:)),all_views.newQ(count,:));
                    
                    %recheck if we have said it is safe
                    if all_views.result(count)==1
                        [obstacle_result,unknown_result]=check_path_for_col(all_views.newQ(count,:),obsticle_points,unknown_points);
                        if ~obstacle_result
                            all_views.result(count)=0;
                        elseif ~unknown_result %we are in unknown space
                            all_views.result(count)=-1;
                        end                        
                    end
                    %could be on known free or unknown spot
                    
                    if all_views.result(count)~=0
                      try [all_views.path(count).valid,all_views.path(count).all_steps]=pathplanner_new(all_views.newQ(count,:),true,false,0,false);end
                        if all_views.path(count).valid==0
                          pathval=pathplanner_water(all_views.newQ(count,:),false);all_views.path(count).valid;all_views.path(count).all_steps=pathval.all_steps;
                        end
                    end
                    
                    %update the count
                    count=count+1;
                end
            end
        end
    end
end

%delete all unneeded ones
all_views=remove_all_views_result0(all_views);

save('all_views.mat','all_views');

%% Display Results - currently commented
toc

%put back global Q to its actual spot
Q=actualQ;