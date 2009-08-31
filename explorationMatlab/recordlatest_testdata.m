%% recordlatest_testdata
% This function can be used to save the latest set of points and the latest
% point cloud range data size

function recordlatest_testdata()
global workspace


global RangeData
hCOM=getappdata(gcf,'hCOM');
aabb = [workspace.min; workspace.max];
hMesh = hCOM.mapHandle.Mesh(aabb);
f = hMesh.FaceData;
v = hMesh.VertexData;
workspace.testdata.nummeshfaces=size(f);
workspace.testdata.nummeshverts=size(v);
workspace.testdata.rangeDataSize=size(RangeData);
workspace.testdata.verts=v;
     