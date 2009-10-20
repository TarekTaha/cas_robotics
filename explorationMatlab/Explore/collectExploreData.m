%this function is used to collect the data for exploration to compare new info
function state_data=collectExploreData(state_data,handles,stepcount)
global workspace Q bestviews %#ok<NUSED>
hCOM=getappdata(gcf,'hCOM');
       
if isempty(state_data)
    state_data.knownweight=calcweight(0);
    state_data.unknownweight=calcweight(0.5);   
    state_data.size_known=size(workspace.knowncoords,1);
    state_data.size_unknown=size(setdiff(round(workspace.unknowncoords/workspace.inc_size),round(workspace.knowncoords/workspace.inc_size),'rows'),1);
    state_data.size_indexedobsticles=size(workspace.indexedobsticles,1);
    state_data.time=0;state_data.starttime=clock;
    state_data.Q=Q;
else
    state_data.knownweight=[state_data.knownweight,calcweight(0)];
    state_data.unknownweight=[state_data.unknownweight,calcweight(0.5)];
    state_data.size_known=[state_data.size_known,size(workspace.knowncoords,1)];
    state_data.size_unknown=[state_data.size_unknown,size(setdiff(round(workspace.unknowncoords/workspace.inc_size),round(workspace.knowncoords/workspace.inc_size),'rows'),1)];
    state_data.size_indexedobsticles=[state_data.size_indexedobsticles,size(workspace.indexedobsticles,1)];
    state_data.time=[state_data.time,etime(clock,state_data.starttime)];
    state_data.Q=[state_data.Q;Q];
end   

% If we are doing testing we want to hold the data
if nargin>1
   if get(handles.testing_checkbox,'value')==1
       testnumber=get(handles.testnumber_edit,'string');
       recordlatest_testdata()
%        testdir=['C:\MATLAB\R2007a\work\Gavin\PhD\PhD_Disertation\Code\Ch8\Stage 1\Test ',num2str(testnumber),'\'];
      testdir='C:\MATLAB\R2007a\work\Gavin\PhD\explorationMatlab\Testing Functions\results\';display('saving to testing directory');
       save([testdir,'AXBAMnC_Test',num2str(testnumber),'Scan',num2str(stepcount-1),'_workspaceSTATE.mat'],'workspace');
       save([testdir,'AXBAMnC_Test',num2str(testnumber),'Scan',num2str(stepcount-1),'_bestviews.mat'],'bestviews');

       hCOM.Surface.StoreSurfaceMap([testdir,'AXBAMnC_Test',num2str(testnumber),'Scan',num2str(stepcount-1),'_newDistanceField.ply']);
       
       save([testdir,'AXBAMnC_Test',num2str(testnumber),'Scan',num2str(stepcount-1),'_bestviews.mat'],'bestviews');
       
       display(['Saving workspace and bestviews to ',testdir]);
  end
end