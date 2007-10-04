%% putIntoVoxels
%
% *Description:* This puts whatever is passed into voxels by dividing by
% the global workspace.inc_size.

%% Function Call
% 
% *Inputs:* points (3 x many cartesian matrix)
% *Returns:* points (3x many cartesian matrix)

function points=putIntoVoxels(points)

%% Variables
global workspace

%% The rounding function
% $$ round(\frac{points}{inc\_size}) \times inc\_size$$

% This function puts the points into voxels 
points=(round(points./workspace.inc_size))*workspace.inc_size;