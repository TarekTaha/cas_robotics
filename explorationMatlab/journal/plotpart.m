%% plotpart
%
% *Description*: This file is passed a ply file and it will go through and plot it out

%% Function Call
%
% *Inputs:* filename (string) :the ply file from database of parts
%
% *Outputs:* NULL
%

function []=plotpart(filename)

%% Variables

%% Input Checks
if nargin<1
  error('need to pass in correct num of vars')
end

%% Functions
try data=plyread([filename,'.ply']);
catch lasterr
end

trisurf(cell2mat(data.face.vertex_indices)+1,data.vertex.x, data.vertex.y, data.vertex.z, 'FaceColor', 'None');
axis equal
    

