%% rotateFigure
%
% *Description:* This function rotates a given figure through a given
% number of degrees by steps of 4 and takes pictures and saves to a png
% file. This can then be made into a movie through something like winmedia
% player

%% Function Call
% 
% *Inputs:* figHandle (double) handle for a figure to be rotated
% degrees (double) num of degrees to be rotated through
% fname (string) the name that will be concaternated at the begining of all
% of the picture files
% *Returns:* Null

function rotateFigure(figHandle, degrees, fname)

%% Variables
nFrames = ceil(degrees/4);

%% Get a grip on figure
figure(figHandle);
[baseAzimuth,baseElevation] = view;

%% Go through num of frames and take a photo at each one
for v=1:nFrames
    view(baseAzimuth - v*6, baseElevation);
    drawnow;
    if 2 < nargin
        print('-dpng', sprintf('%s_%03.0f.png', fname, v));
    end
end
