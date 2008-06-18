function varargout = pathPlannerGUI041008(varargin)
global r submoduleNames;
r = rob_object;
submoduleNames.pathPlanner = 'geneticPlannerHybrid041608';
% PATHPLANNERGUI041008 M-file for pathPlannerGUI041008.fig
%      PATHPLANNERGUI041008, by itself, creates a new PATHPLANNERGUI041008 or raises the existing
%      singleton*.
%
%      H = PATHPLANNERGUI041008 returns the handle to a new PATHPLANNERGUI041008 or the handle to
%      the existing singleton*.
%
%      PATHPLANNERGUI041008('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PATHPLANNERGUI041008.M with the given input arguments.
%
%      PATHPLANNERGUI041008('Property','Value',...) creates a new PATHPLANNERGUI041008 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before pathPlannerGUI041008_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to pathPlannerGUI041008_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help pathPlannerGUI041008

% Last Modified by GUIDE v2.5 14-May-2008 09:48:26

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @pathPlannerGUI041008_OpeningFcn, ...
                   'gui_OutputFcn',  @pathPlannerGUI041008_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

%% --- Executes just before pathPlannerGUI041008 is made visible.
function pathPlannerGUI041008_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to pathPlannerGUI041008 (see VARARGIN)

% Choose default command line output for pathPlannerGUI041008
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using pathPlannerGUI041008.
do_clear(handles);

% UIWAIT makes pathPlannerGUI041008 wait for user response (see UIRESUME)
% uiwait(handles.figure1);
%% Setup variables
function do_clear(handles)

%MANUAL CHECKS: To do before start work
display('Make sure you have done the following');
display('1) Checked the laser is on by running the checklaser.exe file (power cycle if necessary)');
display('2) The ROBOT is powered on');
display('3) The MOTOR power is on');
display('4) Both EMERGENCY STOPs are not on');
display('5) Robot is in automatic mode on both pendant and E-stop control');
display('6) The robocontroller program is running on pendant and recieveing no communication');
display('7) There is no EyeinHand exe running intially in task manager');

% display('Note: You should use the vr world robot not toms, you should also draw up the laser to put on the end as the 7th peice');

display('Pausing so you can do these things');
%pause


% VR world setup
% global myworld
% try close(myworld)
%     delete(myworld)
% end
% 
% vrclear('-force');
% try myworld=vrworld('Robot.WRL');
%     open(myworld);
%     vrfigure(myworld);   
% catch
%     display('Unable to setup vr world');
% end


%sets up the surface map object and then this is used for scanning
try global robmap_h;
    try robmap_h.release;pause(1);end
robmap_h=actxserver('EyeInHand.SurfaceMap');
robmap_h.registerevent(@myhandler);
catch
    %display('EyeInHand Problem: Unable to create surface map')
end

% try and setup platform object
try global platform_h;
    try platform_h.release;pause(1);end
    platform_h = actxserver('EyeInHand.PlatformCommand');
catch
    %display('EyeInHand Problem: Unable to create platform object')
end


%clear the main axis
set(gcf,'CurrentAxes',handles.axes1);
colordef white
cla('reset')
%setlights up
camlight
lighting gouraud

% set(handles.total_info_text,'String','0');set(handles.total_text,'String','0');
% set(handles.best_NBV_listbox,'visible','off');
% set(handles.descripbestview_text,'visible','off');
% set(handles.useRealRobot_checkbox,'Value',0);
% drawnow;

%clear the globals for scan, workspace, robot(r,Q), bestviews, PointData, RangeData
clear global workspace scan bestviews Q r PointData RangeData guiglobal densoobj all_views robot_maxreach;

%Sets up the robot
setuprobot()

global guiglobal
guiglobal.ellipseplots=[];
guiglobal.checkFF=1;
guiglobal.plot_ellipse=0;
guiglobal.plotpath=false;

% Sets up the scanner (scan)
setupscanner();

%this sets up the joint optimisation and NBV optimisation 
setupoptimisationV2()

%this funciton sets up the workspace
%set(handles.dialog_text,'String','Clearing and re-setting up the workspace... Please wait....');
%set(gcf,'CurrentAxes',handles.axes3);

%setupworkspace(get(handles.show_unknownpoints_checkbox,'Value'));
setupworkspace(false);

global all_views
if isempty(all_views)
    uiwait(msgbox('not calculating allviews even though it dosent exist'));
%     display('Having to calculate all_views for exploration, this happens ones only');
     %calc_all_views();
     %load all_views.mat
end

clc;

%set(handles.dialog_text,'String','Setup Complete: Lets Explore');

%% --- Outputs from this function are returned to the command line.
function varargout = pathPlannerGUI041008_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

%% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global r points;
axes(handles.axes1);
cla;
filename = get(handles.edit1, 'String');
%plotdenso(r, [-0.97 -0.46 -0.11 -0.23 -0.13 0]);
%camlight
try 
    load(filename);    
catch
    set(handles.text3, 'String', 'Load Unsuccessful'); 
end
if exist('points')
    set(handles.text3, 'String', 'Load Successful'); 
    plot3(points(:,1),points(:,2),points(:,3),'.b','Markersize', 0.5);
    hold on;
    plotdenso(r, [-0.97 -0.46 -0.11 -0.23 -0.13 0]);
    camlight;axis equal;   
    zoom(2);
    %enable surface selection button
    set(handles.pushbutton4, 'Visible', 'on');
%     [p v vi face facei] = select3d(gco);
%     keyboard
%     select3dtool;
else
    set(handles.text3, 'String', 'Variable Points not Present'); 
end
% popup_sel_index = get(handles.popupmenu1, 'Value');
% switch popup_sel_index
%     case 1
%         plot(rand(5));
%     case 2
%         plot(sin(1:0.01:25.99));
%     case 3
%         bar(1:.5:10);
%     case 4
%         plot(membrane);
%     case 5
%         surf(peaks);
% end


%% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


%% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


%% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'plot(rand(5))', 'plot(sin(1:0.01:25))', 'bar(1:.5:10)', 'plot(membrane)', 'surf(peaks)'});



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


%% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%% --- Executes on key press with focus on pushbutton1 and no controls selected.
function pushbutton1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


%% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%% --------------------------------------------------------------------
function uitoggletool3_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to uitoggletool3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%% --- Executes on button press in pushbutton4. 
% Selecting the blast target area
function pushbutton4_Callback(hObject, eventdata, handles)
global selectPoints;
select3dtool041108
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%% --- Executes on button press in pushbutton5.
% Creating target area
function pushbutton5_Callback(hObject, eventdata, handles)
global selectPoints points targetMeshPoints h;
targetMeshPoints = targetAreaSelection(selectPoints,points);
h.hTargetArea = plot3(targetMeshPoints(:,1),targetMeshPoints(:,2),targetMeshPoints(:,3),'.g','Markersize',0.5);
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%% --- Executes on button press in pushbutton6.
% Clear the selection points and surface area
function pushbutton6_Callback(hObject, eventdata, handles)
global selectPoints targetMeshPoints h
selectPoints = [];
targetMeshPoints = [];
try delete(h.hTargetArea);end;
try
    state = getappdata(gcbf,'select3dtool');
    set(state.marker1,'visible','off');
    set(state.marker2,'visible','off');
    set(state.marker3,'visible','off');
    set(state.marker4,'visible','off');
end
try     
    delete(h.hTargetPoints);
    delete(h.hTargetPointsLine);
end
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%% --- Executes on button press in pushbutton7.
% Target Point Creation, surface making + line plotting
function pushbutton7_Callback(hObject, eventdata, handles)
global targetMeshPoints plane h
plane = surface_making_simple(targetMeshPoints,0.04,false);
for i=1:length(plane)
    x(i)=plane(i).home_point(1);
    y(i)=plane(i).home_point(2);
    z(i)=plane(i).home_point(3);
end
h.hTargetPoints = plot3(x,y,z,'.r');
h.hTargetPointsLine = line(x,y,z,'color','r');    
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%% --- Executes on button press in pushbutton8.
% This button starts the optimisation process, performing PrePoseSelection,
% GA and Post Pose Selection
function pushbutton8_Callback(hObject, eventdata, handles)
global points plane output
iteration = 2500;
show = false;
vidCap = false;

output = geneticPlannerHybrid041608(points, iteration, plane, handles, show, vidCap);
set(handles.text5, 'String', 'Optimisation Complete'); 
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%% --- Executes on button press in pushbutton9.
%This button simulates the path plan on the axes
function pushbutton9_Callback(hObject, eventdata, handles)
global h output r
try     
    delete(h.hTargetPoints);
    delete(h.hTargetPointsLine);
end
pathPlan = output.pathPlan;
jointSequence = output.jointSequence;
for i=1:length(pathPlan)
    x(i)=pathPlan(i).home_point(1);
    y(i)=pathPlan(i).home_point(2);
    z(i)=pathPlan(i).home_point(3);
    hold on
    h.hTargetPoints = plot3(x,y,z,'.r');
    h.hTargetPointsLine = line(x,y,z,'color','r');
    plotdenso(r,jointSequence(i).Q);
    pause(1);
end
 
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


