function saveresultstofile(testnum) 
cd journal
global workspace robot_maxreach 
hCOM=getappdata(gcf,'hCOM');

%save workspace
try save(['test',num2str(testnum),'workspace.mat'],'workspace');
catch; keyboard;end
%save all paths
try save(['test',num2str(testnum),'robot_maxreach.mat'],'robot_maxreach');
catch; keyboard;end
%save mesh
aabb = [-1.5, -1.5, -1; 2, 1.5, 2];
hMesh = hCOM.mapHandle.Mesh(aabb);
hMeshdata.v=hMesh.v;
hMeshdata.f=hMesh.f;

try save(['test',num2str(testnum),'hMesh.mat'],'hMeshdata');
catch; display('Problems saving mesh');
    keyboard;end
 
%save stats
try movefile('../state_data.mat',['test',num2str(testnum),'state_data.mat']);
catch;
    display('statedata dosent exist');
end

%save Xsearchdata
try movefile('../Xsearchdata.mat',['test',num2str(testnum),'Xsearchdata.mat']);
catch;
    display('Xsearchdata.mat dosent exist');
end

ls

cd ..


