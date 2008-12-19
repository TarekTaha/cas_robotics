%% recordlatest_testdata
% This function can be used to save the latest set of points and the latest
% point cloud range data size

function recordlatest_testdata()
global workspace

testnumber=get(handles.testnumber_edit,'string');
global robmap_h RangeData
aabb = [workspace.min; workspace.max];
hMesh = robmap_h.Mesh(aabb);
f = hMesh.FaceData;
v = hMesh.VertexData;
workspace.testdata.nummeshfaces=size(f);
workspace.testdata.nummeshverts=size(v);
workspace.testdata.rangeDataSize=size(RangeData);
workspace.testdata.verts=v;
     