%% exGUI
%
% *Description:*  This is the main GUI file. Goes with the exGUI.fig figure
% file. There are many dependancies for this to work properly including but
% not limited to the setup*.m files and explore.m a NBV* file and the
% planning and movement files if real robot is to be used

%% INITIALISE
function varargout = exGUI(varargin)

%% Variables: Setup GUI
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @exGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @exGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    % Main error catching sequence
    try gui_mainfcn(gui_State, varargin{:});
    catch %#ok<CTCH>
        display('----------------Error-----------------------------');
        display(lasterr); %#ok<LERR>
        display('This error in exGUI resulted in termination')        
        errdetails=lasterror; %#ok<LERR>
        for stackpnt=1:size(errdetails.stack,1)            
            display(errdetails.stack(stackpnt));
            display('.....................................................');
        end   
    end
end
% End initialization code - DO NOT EDIT

%% OPEN
% --- Executes just before exGUI is made visible.
function exGUI_OpeningFcn(hObject, eventdata, handles, varargin)%#ok<INUSL>
% Choose default command line output for exGUI
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);

% Clear the GUI and global variables
do_clear(handles);


%% RETURNS
% --- Outputs from this function are returned to the command line.
function varargout = exGUI_OutputFcn(hObject, eventdata, handles) %#ok<INUSL>
varargout{1} = handles.output;


%% CLOSE
% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
reply = input('Do you want to delete the mesh from memory and release object 1/0 [1]: ');
if isempty(reply)
    reply = 1;
end
if ~reply
    global hCOM %#ok<TLEV>
    hCOM=getappdata(gcf,'hCOM');    
    display('Setting hCOM to be a global var and deleting figure, please get surface map from this variable');
end

%delete coms
COM_Delete_Coms(gcf)

delete(hObject);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Executes on button press in clear_pushbutton.
function clear_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
do_clear(handles);


%% Exploration Functionality (GO pushbutton)
function go_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
explorationMain(handles);
% --- Executes on selection change in best_NBV_listbox.
function best_NBV_listbox_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
plot_NBV(selectionORhandles);
% --- Executes on button press in stopflag_checkbox.
function stopflag_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
set(handles.stopflag_checkbox,'Visible','off')
% --- Executes on button press in find_best_view_pushbutton.
function find_best_view_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
NBV_beta2();
%publish bestviews to GUI
show_best_views_in_GUI(handles)
% --- Executes on button press in remove_selfscan_pushbutton.
function remove_selfscan_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
global workspace
%remove indexed and normal obsticle points within robot FF since not valid
workspace.indexedobsticles=remove_self_scanning(workspace.indexedobsticles);
% --- Executes on button press in threeDmedianfilter_pushbutton.
function threeDmedianfilter_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
threeDMedianFilt();


%% CLASSIFICATION Executes on button press in classify_scan_pushbutton.
function AXBAMnClassify_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
%Go through the num of planes to tryand get to of interest
for i=1:str2double(get(handles.numofdir_AXBAMnC_edit,'string'))
  %do one at a time
  poseclassunknown_Imp_newest(1)
  % save test result
  AXBAMnCtesting(handles);
end
% --- Executes on button press in tilt_scan_pushbutton.
function classify_scan_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
classify_and_update_GUI(handles); 
% --- Executes on button press in AXBAMnClassify_pushbutton.

%% MOVEMENT
function get_current_Js_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
global Q;
if get(handles.useRealRobot_checkbox,'Value'); use_real_robot_GETJs();end
actual_Q=rad2deg(Q);
display(strcat('the current joint state of the real robot is:',num2str(actual_Q)));
set(handles.move_to_J1_edit,'String',num2str(actual_Q(1)));
set(handles.move_to_J2_edit,'String',num2str(actual_Q(2)));
set(handles.move_to_J3_edit,'String',num2str(actual_Q(3)));
set(handles.move_to_J4_edit,'String',num2str(actual_Q(4)));
set(handles.move_to_J5_edit,'String',num2str(actual_Q(5)));
set(handles.move_to_J6_edit,'String',num2str(actual_Q(6)));
function tilt_scan_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
GUI_scanner('tilt',handles)
% --- Executes on button press in rot_scan_pushbutton.
function rot_scan_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
GUI_scanner('pan',handles)
% --- Executes on button press in move_to_pushbutton.
function move_to_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
movetonewQ(handles);
% --- Executes on button press in get_current_Js_pushbutton.


%% PLATFORM COMMANDS
function alloff_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
GUI_platformcontrol('allOff');
function move2start_platform_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
GUI_platformcontrol('MoveToHome');
function move2end_platform_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
GUI_platformcontrol('MoveToEnd');
function moveback_platform_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
GUI_platformcontrol('MoveBackward');
function moveforward_platform_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
GUI_platformcontrol('MoveForward');

%% Plotting and display simple functions
function plot_special_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
global workspace
guiParams=getappdata(gcf,'guiParams');

%can't plot before moving the robot once
if size(workspace.spec_pnts,1)==0
   error('not valid when there are no spec_pnts'); 
else
    hold on;
    set(gcf,'CurrentAxes',handles.axes3);
    if ~isfield(guiParams,'special_pntsplot')
        guiParams.special_pntsplot=plot3(workspace.spec_pnts(:,1),workspace.spec_pnts(:,2),workspace.spec_pnts(:,3),'k*');
        axis vis3d;grid on
    else
        try delete(guiParams.special_pntsplot)
        catch %#ok<CTCH>
            display('cant delete the special points plot');
        end
        guiParams=rmfield(guiParams,'special_pntsplot');
    end
end
setappdata(gcf,'guiParams',guiParams);
% --- Executes on button press in plot_arm_carved_checkbox.
function plot_arm_carved_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
global robot_maxreach
guiParams=getappdata(gcf,'guiParams');
%can't plot before moving the robot once
if size(robot_maxreach.pointcarvedout,1)==0
   error('not valid when there are no points carved out yet'); 
else
    hold on;
    set(gcf,'CurrentAxes',handles.axes3);
    if ~isfield(guiParams,'pointcarvedout')
        guiParams.pointcarvedout=plot3(robot_maxreach.pointcarvedout(:,1),robot_maxreach.pointcarvedout(:,2),robot_maxreach.pointcarvedout(:,3),'b.');
        axis vis3d;grid on
    else
        try delete(guiParams.pointcarvedout)
        catch %#ok<CTCH>
            display('cant delete the pointcarvedout plot');
        end
        guiParams=rmfield(guiParams,'pointcarvedout');
    end
end
setappdata(gcf,'guiParams',guiParams);
% --- Executes on button press in plotmesh_pushbutton.
function plotmesh_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
hCOM=getappdata(gcf,'hCOM');
guiParams=getappdata(gcf,'guiParams');
try delete(guiParams.mesh_h);
catch %#ok<CTCH>
    set(gcf,'CurrentAxes',handles.axes3);
    % if we want to show the whole environment
    if get(handles.all_mesh_checkbox,'value')==1
        aabb = [-20, -20, -20; 20, 20, 20];
    else
        aabb = [-1.5, -1.5, -1; 2, 1.5, 2];
    end
    hMesh = hCOM.Surface.SurfacesInsideBox(aabb(1,:), aabb(2,:));
    f = hMesh.FaceData;
    v = hMesh.VertexData;
    % save('datafile.mat','v');
    hold on;
    guiParams.mesh_h=trisurf(f, v(:,1), v(:,2), v(:,3), 'FaceColor', 'None');
    axis equal
end
setappdata(gcf,'guiParams',guiParams);
% --- Executes on button press in plot_planes_checkbox.
function plot_planes_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
global plane
guiParams=getappdata(gcf,'guiParams');

temp_mew=str2double(get(handles.temp_mew_edit,'String'));

theval=get(hObject,'Value');
if theval && ~isempty(plane)
     temp_plane_plot_handles=plot_planes(plane,temp_mew);
     if isfield(guiParams,'plane_plot_handles')     
         guiParams.plane_plot_handles=[guiParams.plane_plot_handles;temp_plane_plot_handles];
     else
         guiParams.plane_plot_handles=temp_plane_plot_handles;
     end
else
    for current_plane=1:size(guiParams.plane_plot_handles,1); 
        try delete(guiParams.plane_plot_handles(current_plane)); end %#ok<TRYNC>
    end
    guiParams=rmfield(guiParams, 'plane_plot_handles');
end
setappdata(gcf,'guiParams',guiParams);
% --- Executes on button press in make_surface_pushbutton.
function make_surface_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
hCOM=getappdata(gcf,'hCOM');
      
try aabb = [-1.5, -1.5, -1; 2, 1.5, 2];
    hMesh = hCOM.Surface.SurfacesInsideBox(aabb(1,:), aabb(2,:));
    obsticlepoints = hMesh.VertexData(GetImpLevInfo(hMesh.VertexData),:);
catch %#ok<CTCH>
    error('In getting the mesh needed for surface making');
end

temp_mew=str2double(get(handles.temp_mew_edit,'String'));
if size(obsticlepoints)>0
    set(handles.dialog_text,'String','Making planes....');drawnow;
    surface_making_simple(obsticlepoints,temp_mew);
    set(handles.dialog_text,'String','successfully made planes');
else
    set(handles.dialog_text,'String','There are no points to make surfaces out of');
end
% --- Executes on button press in rotate_pushbutton.
function rotate_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
rotate3d;
% --- Executes on button press in ResetView.
function ResetView_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
guiParams=getappdata(gcf,'guiParams');
try delete(guiParams.mesh_h);end %#ok<TRYNC>
set(gcf,'CurrentAxes',handles.axes3);
cla('reset')
camlight
lighting gouraud
axis equal
view(2);
colordef white
setappdata(gcf,'guiParams',guiParams);
% --- Executes on button press in sideon_pushbutton.
function sideon_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
axis equal;
view(3);
% --- Executes on button press in plotexplored_pushbutton.
function plotexplored_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSL,DEFNU>
global workspace
guiParams=getappdata(gcf,'guiParams');
%can't plot before moving the robot once
if size(workspace.knowncoords,1)==0
   error('not valid when there are no explored (known) points'); 
else
    hold on;
    set(gcf,'CurrentAxes',handles.axes3);
    if ~isfield(guiParams,'exploredplot')
        guiParams.exploredplot=plot3(workspace.knowncoords(:,1),workspace.knowncoords(:,2),workspace.knowncoords(:,3),'r.');
        axis vis3d;grid on
    else
        try delete(guiParams.exploredplot)
        catch %#ok<CTCH>
            display('cant delete the exploration plot');
        end
        guiParams=rmfield(guiParams,'exploredplot');
    end
end
setappdata(gcf,'guiParams',guiParams);
% --- Executes on button press in plot_obstacles_pushbutton.
function plot_obstacles_pushbutton_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
global  workspace
guiParams=getappdata(gcf,'guiParams');
if size(workspace.indexedobsticles,1)==0
   error('not valid when there are no indexedobsticles points'); 
else
    set(gcf,'CurrentAxes',handles.axes3)
    hold on;
    if ~isfield(guiParams,'obsticleplot')
        guiParams.obsticleplot=plot3(workspace.indexedobsticles(:,1),workspace.indexedobsticles(:,2),workspace.indexedobsticles(:,3),'g.');
    else
        try delete(guiParams.obsticleplot);
        catch %#ok<CTCH>
            display('cant delete the obsticle plot');
        end
        guiParams=rmfield(guiParams,'obsticleplot');
    end
end
setappdata(gcf,'guiParams',guiParams);
% --- Executes on button press in plot_classified_checkbox.
function plot_classified_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
global workspace classifiedplotHa

if get(hObject,'Value')
    display('Drawing plot')  
    classifiedplotHa=[];
    class_cubesize=workspace.class_cubesize;
    

% NEW WAY    
    classifiedvoxels=find(max(workspace.probofmaterial(:,4:end),[],2)>workspace.classifyProbThreshhold);
    UNclassifiedvoxels=setdiff(1:size(workspace.probofmaterial,1),classifiedvoxels);
    hold on;
    classifiedplotHa(end+1)=plot3(workspace.probofmaterial(UNclassifiedvoxels,1)*class_cubesize,workspace.probofmaterial(UNclassifiedvoxels,2)*class_cubesize,...
    workspace.probofmaterial(UNclassifiedvoxels,3)*class_cubesize,'y','marker','.','markersize',1,'linestyle','none');
      
    [nothing,Materialindex]=max(workspace.probofmaterial(classifiedvoxels,4:end),[],2);
    for curr_mat=unique(Materialindex)'
      IndexofMaterialindex= Materialindex==curr_mat;
      toplot=workspace.probofmaterial(classifiedvoxels(IndexofMaterialindex),1:3)*class_cubesize;
      if curr_mat==1
          classifiedplotHa(end+1)=plot3(toplot(:,1),toplot(:,2),toplot(:,3),'color','k','marker','.','markersize',8,'linestyle','none');         %#ok<AGROW>
      elseif curr_mat==2
          classifiedplotHa(end+1)=plot3(toplot(:,1),toplot(:,2),toplot(:,3),'color',[0.5,0.5,0.5],'marker','.','markersize',8,'linestyle','none');%#ok<AGROW>        
      elseif curr_mat==3
          classifiedplotHa(end+1)=plot3(toplot(:,1),toplot(:,2),toplot(:,3),'color','c','marker','.','markersize',4,'linestyle','none');   %#ok<AGROW>     
      end
      
    end

    drawnow
else
    display('Removing plot')
    for i=1:length(classifiedplotHa);  try delete(classifiedplotHa(i));end; end %#ok<TRYNC>
    clear global classifiedplotHa 
end
% --- Executes on button press in show_unknownpoints_checkbox.
function show_unknownpoints_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
global workspace
guiParams=getappdata(gcf,'guiParams');
if ~get(hObject,'value')
    if isfield (guiParams,'unknownplot')
        for i=1:size(guiParams.unknownplot,2)
            try delete(guiParams.unknownplot(i))
            catch %#ok<CTCH>
                display('Cant delete unknown points plot')
            end
        end
        guiParams=rmfield(guiParams,'unknownplot');
    end
else
    a=round(   workspace.unknowncoords(workspace.lev1unknown   ,:)/workspace.inc_size);
    b=round(     workspace.knowncoords(GetImpLevInfo(workspace.knowncoords)     ,:)/workspace.inc_size);
    c=round(workspace.indexedobsticles(GetImpLevInfo(workspace.indexedobsticles),:)/workspace.inc_size);
    [nothing,index]=setdiff(a,[b;c],'rows');
    
    if ~isempty(index>0)
        guiParams.unknownplot(1)=plot3(workspace.unknowncoords(workspace.lev1unknown(index),1),...
                                   workspace.unknowncoords(workspace.lev1unknown(index),2),...
                                   workspace.unknowncoords(workspace.lev1unknown(index),3),...
                                   '.','Color',[1-workspace.dotweight(1) 1-workspace.dotweight(1) 1]);
    else
        display('No Unknown coordinates to plot');
    end    
end
setappdata(gcf,'guiParams',guiParams);
% --- Executes on button press in show_ellipses_checkbox.
function show_ellipses_checkbox_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
global r Q densoobj
guiParams=getappdata(gcf,'guiParams');
theval=get(hObject,'Value');
plotrob=get(handles.show_robot_checkbox,'Value');

if theval
    guiParams.plot_ellipse=1;
    plotdenso(r, Q, guiParams.checkFF, guiParams.plot_ellipse);
else
    guiParams.plot_ellipse=0;
    plotdenso(r, Q, guiParams.checkFF, guiParams.plot_ellipse);
end
%deletes the robot if it is not needed
if ~plotrob try for piece=1:size(densoobj,2); delete(densoobj(piece).patches); end; end; end;
setappdata(gcf,'guiParams',guiParams);
% --- Executes on button press in show_robot_checkbox.
function show_robot_checkbox_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
global r Q densoobj
guiParams=getappdata(gcf,'guiParams');
theval=get(hObject,'Value');
if theval
    plotdenso(r, Q, guiParams.checkFF, guiParams.plot_ellipse);
    DrawEnv(true);
else
    try for piece=1:size(densoobj,2); delete(densoobj(piece).patches); end; end %#ok<TRYNC>
    DrawEnv(false);
end
% --- Executes on button press in zoom_pushbutton.
function zoom_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
% zoom on;
view(116,18)
keyboard
% --- Executes on button press in imageAq_checkbox.
function imageAq_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
global vid_object
theval=get(hObject,'Value');
if theval
    setupvideo();
else
    imaqreset;
    clear global vid_object
    vid_object=[];    
end
% --- Executes on button press in savefig_pushbutton.
function savefig_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
try 
    AXBAMnCtesting();
catch %#ok<CTCH>
    lasterr %#ok<LERR>
    keyboard
end         
% --- Executes on button press in savePly_pushbutton.
function savePly_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
hCOM=getappdata(gcf,'hCOM');
filename=inputdlg('Enter a file name for ply file');
if isempty(filename)
  filename='you_should_enter_a_filename';
end
directory=pwd;
hCOM.Surface.StoreSurfaceMap([directory,'\',char(filename),'.ply']);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Not Used By Me
% --- Executes during object creation, after setting all properties.
function best_NBV_listbox_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end
% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end
% --- Executes on button press in useRealRobot_checkbox.
function useRealRobot_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
function tilt_scan_deg_edit_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
% --- Executes during object creation, after setting all properties.
function tilt_scan_deg_edit_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Joint 1 edit create and callback
function move_to_J1_edit_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
function move_to_J1_edit_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Joint 2 edit create and callback
function move_to_J2_edit_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
function move_to_J2_edit_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%--- Joint 3 edit create and callback
function move_to_J3_edit_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
function move_to_J3_edit_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Joint 4 edit create and callback
function move_to_J4_edit_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
function move_to_J4_edit_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Joint 5 edit create and callback
function move_to_J5_edit_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
function move_to_J5_edit_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Joint 6 edit create and callback
function move_to_J6_edit_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
function move_to_J6_edit_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes on button press in start_with_default_poses_checkbox.
function start_with_default_poses_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
% --- Executes on button press in animate_move_checkbox.
function animate_move_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
% --- Executes on button press in exact_joints_only_checkbox.
function exact_joints_only_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
% --- Executes during object creation, after setting all properties.
function temp_mew_edit_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
function temp_mew_edit_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% for edit box of how many planes of interest we are going to go through
function AXBAMnC_directs_edit_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
function AXBAMnC_directs_edit_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function numofdir_AXBAMnC_edit_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
function numofdir_AXBAMnC_edit_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function rot_scan_deg_edit_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
% --- Executes during object creation, after setting all properties.
function rot_scan_deg_edit_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes on button press in scanwhilemove_checkbox.
function scanwhilemove_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
% --- Executes on button press in remv_unkn_in_mv_checkbox.
function remv_unkn_in_mv_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
% --- Executes on button press in doclassification_checkbox.
function doclassification_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
% --- Executes during object creation, after setting all properties.
function minClassifications_edit_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function minClassifications_edit_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
global workspace
workspace.minclassifications=str2double(get(hObject,'String'));
% --- Executes during object creation, after setting all properties.
function classification_ration_edit_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function classification_ration_edit_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
global workspace
workspace.classfierthreshhold=str2double(get(hObject,'String'));
function all_mesh_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
function testnumber_edit_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
function plot_classified_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
function testing_checkbox_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
function testnumber_edit_CreateFcn(hObject, eventdata, handles)%#ok<INUSD,DEFNU>

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%% dont know why this is needed, can't find the button for it
% function Untitled_1_Callback(hObject, eventdata, handles)

