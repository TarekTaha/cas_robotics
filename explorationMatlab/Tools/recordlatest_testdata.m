%% recordlatest_testdata
% This function can be used to put into the workspace global variable 
% the latest set of points and the latest
% point cloud range data size

function recordlatest_testdata()
global workspace


global G_scan
hCOM=getappdata(gcf,'hCOM');
aabb = [workspace.min; workspace.max];
hMesh = hCOM.Surface.SurfacesInsideBox(aabb(1,:), aabb(2,:));
f = hMesh.FaceData;
v = hMesh.VertexData;
workspace.testdata.nummeshfaces=size(f);
workspace.testdata.nummeshverts=size(v);
workspace.testdata.rangeDataSize=size(G_scan.RangeData);
workspace.testdata.verts=v;
     