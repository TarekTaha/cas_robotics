%% getdata
% *Description:* 
% 1) Opens the file required, 
% 2) Works out the surfaces due to the verticy data with PCA
% 3) Angle on incidence of each point with the surface
% 4) Updates classifierdata with a new point (angle, intensity, range)
function [vertices, normals, scanrange, source, intensity ,positiondata]=getdata(fileName)

global r config;

show_intensity_image=false;
show_range_edges=false;

hMesh = actxserver('EyeInHand.TriangleMesh');
hMesh.AddRangeGrid([pwd, '\', fileName]);   





positiondata = plyread(fileName);

% get last joint of robot position  
try 
    if config.getposedata
        Q=getjointfromfile(fileName);
        Tr=fkine(r,Q);
    end
end


% get data from hmesh
vertices  = hMesh.VertexData;
normals   = hMesh.NormalData;
scanrange = hMesh.ScannerData(:,1);
intensity = hMesh.ScannerData(:,2);
autogain  = hMesh.ScannerData(:,3);
intensity = intensity ./autogain;
source    = hMesh.ScannerData;

% Can show the intensity image 
intensity_Image_Analysis(positiondata,source,vertices,show_intensity_image,false)

%if you want to do stephens boundardy finding in a range image
if show_range_edges
    range_Map_Analysis(hMesh)
end

end