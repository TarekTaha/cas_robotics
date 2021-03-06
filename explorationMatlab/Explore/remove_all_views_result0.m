%% remove_all_views_result0
%
% *Description*: The all_views variable builds up garbage which are values
% where we have already done it or we have found that it is no longer
% valid, therefore we search for these and remove them since they take up
% a large amount of memory

%% Function Call
%
% *all_views:* all_views (struct) loaded from all_views.mat?
%
% *all_views:* all_views (struct)

function all_views=remove_all_views_result0(all_views)

%% find valid views
resultNot0=(all_views.result==1 | all_views.result==-1);

%% Set each field to the valid (resultNot0) sets
all_views.result=all_views.result(resultNot0);
all_views.newQ=all_views.newQ(resultNot0,:);
all_views.tr=all_views.tr(resultNot0,:,:);
all_views.scanorigin=all_views.scanorigin(resultNot0,:);
all_views.path=all_views.path(resultNot0);
all_views.expectedaddinfo=all_views.expectedaddinfo(resultNot0);

%% Check which views have no new points in them and remove these
resultNot0=true([1,size(all_views.newQ,1)]);
for i=1:size(all_views.newQ,1)
  if isempty(all_views.expectedaddinfo(i).vals)
    resultNot0(i)=false;
  end
end

%% Set each field to the valid (resultNot0) sets
if ~isempty(find(resultNot0==false,1))
  all_views.result=all_views.result(resultNot0);
  all_views.newQ=all_views.newQ(resultNot0,:);
  all_views.tr=all_views.tr(resultNot0,:,:);
  all_views.scanorigin=all_views.scanorigin(resultNot0,:);
  all_views.path=all_views.path(resultNot0);
  all_views.expectedaddinfo=all_views.expectedaddinfo(resultNot0);
end


