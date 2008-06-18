%% putInVoxels_matlab
%
% *Description:* If the C code dosen't work then this could be used
% This puts whatever is passed into voxels by dividing by the global
% workspace.inc_size. 

%% Function Call
% 
% *Inputs:* 
%
% _points_ (3 x many cartesian matrix)
% _inc_size_ ( 1 x 1 double) size of voxels
%
% *Returns:* 
%
% _points_ (3x many cartesian matrix)

function points=putInVoxels_matlab(points,inc_size)

%% The rounding function
% $$ round(\frac{points}{inc\_size}) \times inc\_size$$

% This function puts the points into voxels 
points=(round(points/inc_size))*inc_size;
points=unique(points,'rows');