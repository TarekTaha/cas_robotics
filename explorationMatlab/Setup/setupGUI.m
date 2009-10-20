%This is for seting up the exGUI

function setupGUI(handles)

% Clear the main axis
set(gcf,'CurrentAxes',handles.axes3);
colordef white
cla('reset')
%setlights up
camlight
lighting gouraud

set(handles.total_info_text,'String','0');set(handles.total_text,'String','0');
set(handles.best_NBV_listbox,'visible','off');
set(handles.descripbestview_text,'visible','off');
set(handles.useRealRobot_checkbox,'Value',0);
drawnow;

% Setup GUI variables
guiParams.ellipseplots=[];
guiParams.checkFF=1;
guiParams.plot_ellipse=0;
guiParams.plotpath=false;
setappdata(gcf,'guiParams',guiParams);

%this funciton sets up the workspace
set(handles.dialog_text,'String','Setup Complete: Lets Explore');
set(gcf,'CurrentAxes',handles.axes3);

