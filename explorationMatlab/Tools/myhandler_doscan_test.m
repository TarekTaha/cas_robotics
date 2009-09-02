function myhandler_doscan_test(varargin)
global   IntensityData  PointData
if strcmpi(varargin{end}, 'ScanComplete')
scan = varargin{3}; % A COM handle to an IRangeScan object
IntensityData(end+1,:) = scan.IntensityData;
PointData(end+1,:,:) = scan.PointData;
scan.release;
elseif strcmpi(varargin{end}, 'TiltComplete')
disp('Tilt complete'); % Change this to something useful
end
end