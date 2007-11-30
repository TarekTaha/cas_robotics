%% order_bestviews
%
% *Description:*  Using both their additional information and their joint movement from Q
% determine which is more desirable, give weightings to both joint movement

%% Function Call
%
% *Inputs:* 
%
% _redo_nbv_vol_ (binary) wether or not to redo the nbv volume and pose checking
% for each on the bestviews
%
% *Returns:* NULL

function order_bestviews(redo_nbv_vol)

%% Variables
global bestviews

if nargin==0
    redo_nbv_vol=0;
end

%% Calculate new nbv_vol if required for each pose
if redo_nbv_vol
    % Remove impossible poses passed upon new info
    current_count=1;
    for cur_view=1:size(bestviews,2)
        if check_path_for_col(bestviews(cur_view).Q)
            temp_bestviews(current_count)=bestviews(cur_view);
            current_count=current_count+1;
        end
    end
    bestviews=temp_bestviews;
    unknownweight=calunknownweight();
    
    %go through remaining and update the expected info/addinfo(weight)
    for cur_view=1:size(bestviews,2)
        bestviews(cur_view).expectedaddinfo=nbv_volume(bestviews(cur_view).tr,bestviews(cur_view).Q);
        bestviews(cur_view).addinfo=getweighted_addinfo(bestviews(cur_view).expectedaddinfo)/unknownweight;
    end        
end


%% Go through each weight and collect for a sort
overallweight=zeros([1,size(bestviews,2)]);
jointmoveweight=zeros([1,size(bestviews,2)]);addinfoweight=zeros([1,size(bestviews,2)]);
% Collect all weights so they can be sorted
for cur_view=1:size(bestviews,2)
    calculate_NBV_overallweight(cur_view);
    overallweight(cur_view) = bestviews(cur_view).overall;
    jointmoveweight(cur_view) = bestviews(cur_view).jointmoveweight;
    addinfoweight(cur_view) = bestviews(cur_view).addinfoweight;
end

%% Sort the remaining bestviews
[vals,index]=sort(overallweight,'descend');

%% Remake the bestviews global variable
clear temp_bestviews;
current_count=1;
for cur_view=index
    temp_bestviews(current_count)=bestviews(cur_view);
    current_count=current_count+1;
end
bestviews=temp_bestviews;

%% Display: info about the averages and such
display(strcat('Aver_joint=',num2str(sum(jointmoveweight)/length(jointmoveweight)),...
             ', Aver_info=',num2str(sum(addinfoweight)/length(addinfoweight)),...
             ', Max_joint=',num2str(bestviews(1).jointmoveweight),...
             ', Max_info=',num2str(bestviews(1).addinfoweight),...
             ', TotalAv=',num2str(sum(overallweight)/length(overallweight)),...
             '...J_Contrib%:',num2str((sum(jointmoveweight)/sum(addinfoweight))*100),'%'));
% figure(3)
% subplot(2,1,1);plot(overallweight,'r.');hold on;plot(jointmoveweight,'g.');hold on;plot(addinfoweight,'b.')
% subplot(2,1,2);plot(overallweight(index),'r.');hold on;plot(jointmoveweight(index),'g.');hold on;plot(addinfoweight(index),'b.');
% grid on