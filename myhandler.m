%% myhandler
%
% *Description:*  This function is used to handle the data coming from the
% scanning robot object. It saves data into the variables

%% Function Call
%
% * *Inputs:* 
% varargin (variable) data returned by C++ com object
% * *Returns:* Null

function myhandler(varargin)

%% Variables
global PoseData RangeData IntensityData PointData

%% Check the value passed in
if strcmpi(varargin{end}, 'ScanComplete')
     scan = varargin{3}; % A COM handle to an IRangeScan object
     RangeData(end+1,:) = scan.RangeData;
     IntensityData(end+1,:) = scan.IntensityData;
     PointData(end+1,:,:) = scan.PointData;
     if scan.HasPose
         PoseData(end+1,:,:) = scan.Pose;
     end
     scan.release;
elseif strcmpi(varargin{end}, 'TiltComplete')
     disp('Tilt complete'); % Change this to something useful
end
