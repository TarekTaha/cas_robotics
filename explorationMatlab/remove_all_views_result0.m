function all_views=remove_all_views_result0(all_views)

%find valid views
resultNot0=find(all_views.result==1 | all_views.result==-1);

all_views.result=all_views.result(resultNot0);
all_views.newQ=all_views.newQ(resultNot0,:);
all_views.tr=all_views.tr(resultNot0,:,:);
all_views.scanorigin=all_views.scanorigin(resultNot0,:);
all_views.path=all_views.path(resultNot0);
all_views.expectedaddinfo=all_views.expectedaddinfo(resultNot0);



