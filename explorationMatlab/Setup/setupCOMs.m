%This function calls the setup functions in COM directory 
function setupCOMs(hFigure)
global workspace

%% Logger activation and clock resync
try
  hCOM.App = actxserver('EyeInHand.Application');
  setappdata(hFigure,'hCOM',hCOM);
catch %#ok<CTCH>
  error('EyeInHand.Application problem...');
end;

%% EM switch & platform
try
  hCOM.Platform=actxserver('EyeInHand.PlatformState');
  hCOM.PlatformCommand=actxserver('EyeInHand.PlatformCommand');
  setappdata(hFigure,'hCOM',hCOM);
catch %#ok<CTCH>
  error('PlatformState problem...');
end;

%% Denso state
try
  hCOMdenso=actxserver('EyeInHand.DensoState');
  hCOMdenso.Start;
  hCOM.Denso=hCOMdenso;
  % Used to continually monitor the joint state
  hCOM.Denso.registerevent(@updateQlistener);
  setappdata(hFigure,'hCOM',hCOM);
catch %#ok<CTCH>
  disp('DensoState problem...');
end;
  
%% Denso query
try
  hCOM.Query=actxserver('EyeInHand.DensoQuery');
  setappdata(hFigure,'hCOM',hCOM);
catch %#ok<CTCH>
  disp('DensoQuery problem...');
end;
  
%% Denso command
try
   hCOM.Command=actxserver('EyeInHand.DensoCommand');
   setappdata(hFigure,'hCOM',hCOM);
catch %#ok<CTCH>
  disp('DensoCommand problem...');
end;

%% LASER/SURFACE/OCCUPANCY
try
  %size of workspace
  passed_parameter_struct.aabbExtent = [workspace.min(1), workspace.min(2), workspace.min(3);workspace.max(1), workspace.max(2), workspace.max(3)];
  %size of robot
  passed_parameter_struct.aabbSelf = [workspace.robotsize(1,1), workspace.robotsize(2,1), workspace.robotsize(3,1); workspace.robotsize(1,2), workspace.robotsize(2,2), workspace.robotsize(3,2)];
  %the voxel size
  passed_parameter_struct.voxelsize=workspace.inc_size*1000;
  % I don't want to display the occupancy grit info
  passed_parameter_struct.show_parameters=false;
  
  if ~COM_PrepareScanner(hFigure,passed_parameter_struct);
    error(' could not prepare scanner, surfacemap or occupancy grid');
  end
catch %#ok<CTCH>
  disp('OccupancyMap problem...');
end;
