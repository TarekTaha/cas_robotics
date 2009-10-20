function GUI_scanner(option,handles)

global r Q
%% TILT scan
if strcmp(option,'tilt');
    
    if get(handles.useRealRobot_checkbox,'Value')==0
        doscan();
    else
        deg2scan=str2double(get(handles.tilt_scan_deg_edit,'String'));
        use_real_robot_SCAN(deg2scan);
        organise_data();
        guiParams=getappdata(gcf,'guiParams');   
        plotdenso(r, Q, guiParams.checkFF, guiParams.plot_ellipse);    
    end
%% PAN scan
elseif strcmp(option,'pan');
    if get(handles.useRealRobot_checkbox,'Value')==0
        error('Not avaialbe in simulation mode');
    else
        deg2scan=str2double(get(handles.rot_scan_deg_edit,'String'));
        use_real_robot_SCANandMOVE(deg2scan);
        organise_data()
        use_real_robot_GETJs();
        guiParams=getappdata(gcf,'guiParams');
        plotdenso(r, Q, guiParams.checkFF, guiParams.plot_ellipse);
    end
else
    error('invalid option passed');
end
