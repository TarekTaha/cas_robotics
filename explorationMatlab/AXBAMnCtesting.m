%% AXBAMnCtesting
%
% *Description*: This is for plotting the current state of the output of
% the classification

%% Function Call
%
% *Inputs:* 
%
% _plotdirectedC_ (binary) Whether or not to plot the direction
%
% *Returns:* NULL

function AXBAMnCtesting()%plotdirectedC)

%% Variables
plot_the_current_mesh_and_robot=false;
plot_the_current_classification_and_robot=false;
save_the_workspace_datafile_at_that_point=true;

%plot the current mesh and robot
global r Q robmap_h workspace AXBAMnCtestdata
if nargin==0
%     plotdirectedC=input('Do you want to plot the directed classification? 1=Yes, 0=No');
%     try if plotdirectedC~=1 && plotdirectedC~=0
%         plotdirectedC=false;
%         end
%     catch;
%         plotdirectedC=false;
%     end
end

% testnum= input('Please enter TEST number','s');
% scannum= input('Please enter SCAN number','s');
% plotunknown=input('Do you want to plot the unknown voxels 1=yes, 0=no');
testnum=1;
testdir=['C:\MATLAB\R2007a\work\Gavin\PhD\PhD_Disertation\Code\Ch8\Stage 2\Test ',num2str(testnum),'\'];
%as long as there is nothing else in the directory the numbering should be fine
filesInDir=dir(testdir);
scannum= (size(filesInDir,1)-2)/2+1;
plotunknown=0;
    
% testdir='C:\Documents and Settings\RTA computer\My Documents\My Pictures\';

%% Save the latest pointcloud size, info and verts
recordlatest_testdata();

%% Plot the current mesh and robot
% if plot_the_current_mesh_and_robot
%   figure
%   plotdenso(r,Q)
%   camlight
%   aabb = [-1.5, -1.5, -1; 2, 1.5, 2];
%   hMesh = robmap_h.Mesh(aabb);
%   f = hMesh.FaceData;
%   v = hMesh.VertexData;
%   hold on;
%   guiglobal.mesh_h=trisurf(f, v(:,1), v(:,2), v(:,3), 'FaceColor', 'None');
%   axis equal
%   view(116,18)
%   saveas(gcf,[testdir,'AXBAMnC_Test',testnum,'Scan',scannum,'_meshNrobot.fig']);
%   close gcf
% end
% 
% %% plot the current classification and robot
% if plot_the_current_classification_and_robot
%   figure
%   plotdenso(r,Q)
%   camlight
%   class_cubesize=workspace.class_cubesize;
% 
%       %determine the unknown places and the know metal or wood
%   sumofclass=workspace.ocgrid(:,4)+workspace.ocgrid(:,5);
%   warning('off','MATLAB:divideByZero')
%   try classifiedvoxels=find(sumofclass>=workspace.minclassifications &...
%                        (workspace.ocgrid(:,4)./workspace.ocgrid(:,5)>workspace.classfierthreshhold |...
%                         workspace.ocgrid(:,5)./workspace.ocgrid(:,4)>workspace.classfierthreshhold));end
%   try UNclassifiedvoxels=find(sumofclass<workspace.minclassifications | ...
%                        (workspace.ocgrid(:,4)./workspace.ocgrid(:,5)<=workspace.classfierthreshhold &...
%                         workspace.ocgrid(:,5)./workspace.ocgrid(:,4)<=workspace.classfierthreshhold));end
%   warning('on','MATLAB:divideByZero')
% 
%       %now plot this
%   hold on;
%   if plotunknown
%       try plot3(workspace.ocgrid(UNclassifiedvoxels,1)*class_cubesize,...
%             workspace.ocgrid(UNclassifiedvoxels,2)*class_cubesize,...
%             workspace.ocgrid(UNclassifiedvoxels,3)*class_cubesize,'y','marker','.','markersize',0.5,'linestyle','none');end
%   end
%   %plot metal and wood voxels
% 
%   metalvoxels=workspace.ocgrid(classifiedvoxels(workspace.ocgrid(classifiedvoxels,4)>workspace.ocgrid(classifiedvoxels,5)),1:3);
%   if size(metalvoxels,1)>0
%       try plot3(metalvoxels(:,1)*class_cubesize,metalvoxels(:,2)*class_cubesize,metalvoxels(:,3)*class_cubesize,'r.');end
%   end
% 
%   woodvoxels=workspace.ocgrid(classifiedvoxels(workspace.ocgrid(classifiedvoxels,4)<workspace.ocgrid(classifiedvoxels,5)),1:3);
%   if size(woodvoxels,1)>0
%       try plot3(woodvoxels(:,1)*class_cubesize,woodvoxels(:,2)*class_cubesize,woodvoxels(:,3)*class_cubesize,'b.');end
%   end
% 
% 
%   axis equal
%   view(116,18)
%   saveas(gcf,[testdir,'AXBAMnC_Test',testnum,'Scan',scannum,'_classNrobot.fig']);
%   close gcf
% end

%% save the workspace data file at that point
if save_the_workspace_datafile_at_that_point
  save([testdir,'AXBAMnC_Test',num2str(testnum),'Scan',num2str(scannum),'_workspaceSTATE.mat'],'workspace');
  save([testdir,'AXBAMnC_Test',num2str(testnum),'Scan',num2str(scannum),'_workspacePREV_STATE.mat'],'AXBAMnCtestdata');
end

%% plot directed C

% if plotdirectedC && ~isempty(AXBAMnCtestdata)
% %after (uses previous steps found metal and wood)
%     figure
%     plotdenso(r,AXBAMnCtestdata.newQ)
%     camlight
%     plotfill=false;
%     plot_planes(AXBAMnCtestdata.plane_aimed_at,AXBAMnCtestdata.mew,plotfill);        
%     hold on;
%     plot3(AXBAMnCtestdata.plane_aimed_at.home_point(1),AXBAMnCtestdata.plane_aimed_at.home_point(2),AXBAMnCtestdata.plane_aimed_at.home_point(3),'b*');
%     plot3([AXBAMnCtestdata.plane_aimed_at.home_point(1),AXBAMnCtestdata.plane_aimed_at.home_point(1)+AXBAMnCtestdata.plane_aimed_at.normal_by_eigenval(1)/10],...
%           [AXBAMnCtestdata.plane_aimed_at.home_point(2),AXBAMnCtestdata.plane_aimed_at.home_point(2)+AXBAMnCtestdata.plane_aimed_at.normal_by_eigenval(2)/10],...
%           [AXBAMnCtestdata.plane_aimed_at.home_point(3),AXBAMnCtestdata.plane_aimed_at.home_point(3)+AXBAMnCtestdata.plane_aimed_at.normal_by_eigenval(3)/10],'b');                                   
%     if plotunknown   
%         try plot3(workspace.ocgrid(UNclassifiedvoxels,1)*class_cubesize,...
%           workspace.ocgrid(UNclassifiedvoxels,2)*class_cubesize,...
%           workspace.ocgrid(UNclassifiedvoxels,3)*class_cubesize,'y','marker','.','markersize',0.5,'linestyle','none');end
%     end
%     if size(metalvoxels,1)>0;try plot3(metalvoxels(:,1)*class_cubesize,metalvoxels(:,2)*class_cubesize,metalvoxels(:,3)*class_cubesize,'r.');end; end
%     if size(woodvoxels,1)>0;try plot3(woodvoxels(:,1)*class_cubesize,woodvoxels(:,2)*class_cubesize,woodvoxels(:,3)*class_cubesize,'b.');end; end
% 
%     axis equal
%     view(116,18)
%     saveas(gcf,[testdir,'AXBAMnC_Test',testnum,'Scan',scannum,'_directedscanAFTER.fig']);
%     close gcf
%     display(['Size of ocgrid IS ',num2str(size(workspace.ocgrid)),' with unknown number=',num2str(size(UNclassifiedvoxels))])
%     
% % before
%     figure
%     plotdenso(r,AXBAMnCtestdata.newQ)
%     camlight
%     plotfill=false;
%     plot_planes(AXBAMnCtestdata.plane_aimed_at,AXBAMnCtestdata.mew,plotfill);        
%     hold on;
%     plot3(AXBAMnCtestdata.plane_aimed_at.home_point(1),AXBAMnCtestdata.plane_aimed_at.home_point(2),AXBAMnCtestdata.plane_aimed_at.home_point(3),'b*');
%     plot3([AXBAMnCtestdata.plane_aimed_at.home_point(1),AXBAMnCtestdata.plane_aimed_at.home_point(1)+AXBAMnCtestdata.plane_aimed_at.normal_by_eigenval(1)/10],...
%           [AXBAMnCtestdata.plane_aimed_at.home_point(2),AXBAMnCtestdata.plane_aimed_at.home_point(2)+AXBAMnCtestdata.plane_aimed_at.normal_by_eigenval(2)/10],...
%           [AXBAMnCtestdata.plane_aimed_at.home_point(3),AXBAMnCtestdata.plane_aimed_at.home_point(3)+AXBAMnCtestdata.plane_aimed_at.normal_by_eigenval(3)/10],'b');                                   
%     Pworkspace=AXBAMnCtestdata.Pworkspace;
%         %determine the unknown places and the know metal or wood
%     sumofclass=Pworkspace.ocgrid(:,4)+Pworkspace.ocgrid(:,5);
%     warning('off','MATLAB:divideByZero')
%     try classifiedvoxelsP=find(sumofclass>=Pworkspace.minclassifications &...
%                          (Pworkspace.ocgrid(:,4)./Pworkspace.ocgrid(:,5)>Pworkspace.classfierthreshhold |...
%                           Pworkspace.ocgrid(:,5)./Pworkspace.ocgrid(:,4)>Pworkspace.classfierthreshhold));end
%     try UNclassifiedvoxelsP=find(sumofclass<Pworkspace.minclassifications | ...
%                          (Pworkspace.ocgrid(:,4)./Pworkspace.ocgrid(:,5)<=Pworkspace.classfierthreshhold &...
%                           Pworkspace.ocgrid(:,5)./Pworkspace.ocgrid(:,4)<=Pworkspace.classfierthreshhold));end
%     warning('on','MATLAB:divideByZero')
% 
%     %now plot this
%     hold on;
%     if plotunknown
%         try plot3(Pworkspace.ocgrid(UNclassifiedvoxelsP,1)*class_cubesize,...
%               Pworkspace.ocgrid(UNclassifiedvoxelsP,2)*class_cubesize,...
%               Pworkspace.ocgrid(UNclassifiedvoxelsP,3)*class_cubesize,'y','marker','.','markersize',0.5,'linestyle','none');end
%     end
%     %plot metal and wood voxels
% 
%     metalvoxels=Pworkspace.ocgrid(classifiedvoxelsP(Pworkspace.ocgrid(classifiedvoxelsP,4)>Pworkspace.ocgrid(classifiedvoxelsP,5)),1:3);
%     if size(metalvoxels,1)>0
%         try plot3(metalvoxels(:,1)*class_cubesize,metalvoxels(:,2)*class_cubesize,metalvoxels(:,3)*class_cubesize,'r.');end
%     end
% 
%     woodvoxels=Pworkspace.ocgrid(classifiedvoxelsP(Pworkspace.ocgrid(classifiedvoxelsP,4)<Pworkspace.ocgrid(classifiedvoxelsP,5)),1:3);
%     if size(woodvoxels,1)>0
%         try plot3(woodvoxels(:,1)*class_cubesize,woodvoxels(:,2)*class_cubesize,woodvoxels(:,3)*class_cubesize,'b.');end
%     end
% 
%     
%     axis equal
%     view(116,18)
%     saveas(gcf,[testdir,'AXBAMnC_Test',testnum,'Scan',scannum,'_directedscanBEFORE.fig']);
%     close gcf
%     display(['Size of ocgrid WAS ',num2str(size(Pworkspace.ocgrid)),' with unknown number=',num2str(size(UNclassifiedvoxelsP))])
% %     save([testdir,'AXBAMnC_Test',testnum,'Scan',scannum,'_workspacePREV_STATE.mat'],'Pworkspace');
% end
