function setupCOMs()
global workspace
[hCOM]=COM_Create_Coms(gcf);

%sets up the surface map object and then this is used for scanning
[hCOM.Surface]=COM_CreateSurfaceMap;
hCOM.Laser.AddObserver(hCOM.Surface);

% Used to continually monitor the joint state
hCOM.Denso.registerevent(@updateQlistener);


%size of workspace
aabbExtent = [workspace.min(1), workspace.min(2), workspace.min(3);...
  workspace.max(1), workspace.max(2), workspace.max(3)];
%size of robot
aabbSelf = [workspace.robotsize(1,1), workspace.robotsize(2,1), workspace.robotsize(3,1);...
  workspace.robotsize(1,2), workspace.robotsize(2,2), workspace.robotsize(3,2)];

%create occupancy handle
[hCOM.occHandle]=COM_CreateOccupancyMap(workspace.inc_size*1000,aabbExtent,aabbSelf);

hCOM.Laser.AddObserver(hCOM.occHandle)


%add all hCOM to figure handle
setappdata(gcf,'hCOM',hCOM);