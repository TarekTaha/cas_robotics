%% exGUI
%
% *Description:*  This is the main GUI file. Goes with the exGUI.fig figure
% file. There are many dependancies for this to work properly including but
% not limited to the setup*.m files and explore.m a NBV* file and the
% planning and movement files if real robot is to be used

%% Function Call
%
% *Inputs:* 
%
% _varargin_ (unknown) values passed in
%
% *Returns:* 
% 
% _varargout_ (unknown) values returned

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
    catch
        display('----------------Error-----------------------------');
        display(lasterr);
        display('This error in exGUI resulted in termination')        
        errdetails=lasterror;
        for stackpnt=1:size(errdetails.stack,1)            
            display(errdetails.stack(stackpnt));
            display('.....................................................');
        end   
    end
end
% End initialization code - DO NOT EDIT

% --- Executes just before exGUI is made visible.
function exGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% Choose default command line output for exGUI
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);
% Clear the GUI and global variables
do_clear(handles);

% --- Outputs from this function are returned to the command line.
function varargout = exGUI_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;

%% Executes on button press in clear_pushbutton.
function clear_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
clc
do_clear(handles);

% this is the do_clear function sets up most stuff
function do_clear(handles)

%sets up the surface map object and then this is used for scanning
try global robmap_h;
robmap_h=actxserver('EyeInHand.SurfaceMap');
robmap_h.registerevent(@myhandler);
catch
    display('Unable to connect to EyeInHand')
end

%clear the main axis
set(gcf,'CurrentAxes',handles.axes3);
cla('reset')
set(handles.total_info_text,'String','0');set(handles.total_text,'String','0');
set(handles.best_NBV_listbox,'visible','off');
set(handles.descripbestview_text,'visible','off');
set(handles.useRealRobot_checkbox,'Value',0);
drawnow;

%clear the globals for scan, workspace, robot(r,Q), bestviews, PointData, RangeData
clear global workspace scan bestviews Q r PointData RangeData guiglobal densoobj;

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
setupoptimisation()

%this funciton sets up the workspace
set(handles.dialog_text,'String','Clearing and re-setting up the workspace... Please wait....');
set(gcf,'CurrentAxes',handles.axes3);

setupworkspace(get(handles.show_unknownpoints_checkbox,'Value'));
global workspace;

set(handles.dialog_text,'String','Setup Complete: Lets Explore');


%% Exploration Functionality
% --- Executes on button press in go_pushbutton.
function go_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>

%%%CHOOSE THE CURRENT TEST CASE
current_test_case=2;
show_new_info_details=false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% profile clear; profile on;
global robot_maxreach optimise workspace scan

%update Q to the latest actual value of the robot
if get(handles.useRealRobot_checkbox,'Value')==1;  use_real_robot_GETJs();end   

%start data collection
state_data=collectdata([]);
useNBV=false;
set(gcf,'CurrentAxes',handles.axes3)

%% intial safe pose default scans
%since we wish to choose our own default start points
for stepcount=2:size(robot_maxreach.default_Q,1)+1
    want_to_continue=true;
    while want_to_continue && scan.tries<size(robot_maxreach.default_Q,1)
        try explore(handles,useNBV); break; 
        catch; display(lasterr); 
            want_to_continue=input('Type (1) to continue, (2) for keyboard command, (0) to exit\n');            
            if want_to_continue==2; keyboard; end
            if want_to_continue==0; error('User chose to exit');end
            if get(handles.useRealRobot_checkbox,'Value')==1;  use_real_robot_GETJs();end   
        end; 
    end
    state_data=collectdata(state_data); 
end
    
%% TEST 1 - random points
if current_test_case==1
    global r bestviews
    qlimits=r.qlim;
    for stepcount=1:20
        %%This is the random find next point
        pathfound=false;
        while ~pathfound
            useNBV=true;
            tempendQs=rand(1,6);newQ=[];
            for i=1:6
                newQ=[newQ,tempendQs(:,i)*(-qlimits(i,1)+qlimits(i,2))+qlimits(i,1)];
            end
            tr=fkine(r,newQ);
            tr_indexed=round(tr(1:3,4)/workspace.inc_size)*workspace.inc_size;
            if ~isempty(find(tr_indexed(1)==workspace.knowncoords(:,1)&...
                             tr_indexed(2)==workspace.knowncoords(:,2)&...
                             tr_indexed(3)==workspace.knowncoords(:,3), 1))                  
                [pathfound,all_steps]=pathplanner(newQ);
                if pathfound; plotdenso(r,newQ,false,false); end;
            end
        end

        bestviews.scanorigin=tr(1:3,4)';
        bestviews.chosenview=sum(tr(1:3,1:3));
        bestviews.tr=tr;            
        bestviews.Q=newQ;
        bestviews.all_steps=all_steps;
        bestviews.valid=true;
        try explore(handles,useNBV,1);
        catch display(lasterr);
        end
        %data collection
        state_data=collectdata(state_data);
    end

%% TEST 2 - Non points -using algorithm
elseif current_test_case>1
    %now go through and get NBV and then use them to explore
    for stepcount=stepcount+1:10;
        %which exploration method to use
        if current_test_case==2;NBV_beta();
        elseif current_test_case==3; NBV();
        end
        
        current_bestview=1;
        max_bestviews_togothrough=optimise.valid_max*1/4;
        global bestviews;        
        while current_bestview<=max_bestviews_togothrough && size(bestviews,2)>=1
            %size(bestviews,2)>optimise.valid_max*3/4
            current_bestview=current_bestview+1;
            
            useNBV=true;
            want_to_continue=true;
            while want_to_continue; 
                try if movetonewQ(handles,bestviews(1).Q*180/pi);
                        scan.done_bestviews_orfailed=[scan.done_bestviews_orfailed;bestviews(1).Q];
                        explore(handles,useNBV,1);break;                        
                    end
                    want_to_continue=0;                        
                catch; display(lasterr);
                    want_to_continue=input(' Type (1) to continue, (2) for keyboard command, (0) to exit\n');            
                    if want_to_continue==2; keyboard; end
                    if want_to_continue==0; error('User chose to exit');end
                    if get(handles.useRealRobot_checkbox,'Value')==1;  use_real_robot_GETJs();end   
                end;
            end                

if show_new_info_details            
%Plotting and dispalying what we expected compared to what we got
display(strcat('The size of the expected infor was:',num2str(size(bestviews(1).expectedaddinfo)),', While the actual size was:',num2str(size(workspace.newestknownledge)),...
    ', The set difference was:',num2str(size(setdiff(bestviews(1).expectedaddinfo,workspace.newestknownledge,'rows'))),...
    ', The weighted addinfo is:',num2str(bestviews(1).addinfo),'. And the overall weight was:',num2str(bestviews(1).overall)));

    temp=plot3(bestviews(1).expectedaddinfo(:,1),bestviews(1).expectedaddinfo(:,2),bestviews(1).expectedaddinfo(:,3),'r.');
    temp2=plot3(workspace.newestknownledge(:,1),workspace.newestknownledge(:,2),workspace.newestknownledge(:,3),'g.');
    temp3=setdiff(bestviews(1).expectedaddinfo,workspace.newestknownledge,'rows');
    temp4=plot3(temp3(:,1),temp3(:,2),temp3(:,3),'b.');
    pause(2);
    delete(temp); delete(temp4);
    pause(2);delete(temp2); 
end

            if size(bestviews,2)>1  
                %Resize bestviews so as to get rid of the first element
                clear temp_bestviews;
                for cur_view=2:size(bestviews,2); temp_bestviews(cur_view-1)=bestviews(cur_view);end
                    bestviews=temp_bestviews;

                    state_data=collectdata(state_data); 

                    redo_nbv_vol=true;
                    order_bestviews(redo_nbv_vol);
            else
                bestviews=[];
            end
        end
    end
end


%Plotting the results
figure;
subplot(3,1,1);plot(state_data.knownweight);grid on; title('Known Weight');
subplot(3,1,2);plot(state_data.size_known);grid on; title('Total Known Points');
subplot(3,1,3);plot(state_data.time);grid on; title('Time Taken');

% profile off; profile viewer;

% --- Executes on selection change in best_NBV_listbox.
function best_NBV_listbox_Callback(hObject, eventdata, handles) %#ok<DEFNU>
global bestviews;
selection=get(handles.best_NBV_listbox,'value');
if bestviews(selection).valid
    
%%%%%%%%delete this plotting stuff    %%%%%%%%%%%
%plot the scanorigin
    tempplothandle=plot3(bestviews(selection).scanorigin(1),bestviews(selection).scanorigin(2),bestviews(selection).scanorigin(3),'r*');
    hold on;
%plot the direction
    pointAT=bestviews(selection).chosenview+bestviews(selection).scanorigin;
    tempplothandle= [tempplothandle,plot3([bestviews(selection).scanorigin(1),pointAT(1)],[bestviews(selection).scanorigin(2),pointAT(2)],[bestviews(selection).scanorigin(3),pointAT(3)])];   
    uiwait(msgbox('have a look then press ok'))
    try for i=1:length(tempplothandle); delete(tempplothandle(i)); end; end;
%plot the new info    
    tempplothandle= plot3(bestviews(selection).expectedaddinfo(:,1),bestviews(selection).expectedaddinfo(:,2),bestviews(selection).expectedaddinfo(:,3),'y.');
    uiwait(msgbox('have a look then press ok'))
    try delete(tempplothandle); end;
%%%%%%%%ENDdelete this plotting stuff    %%%%%%%%%%%
    
    useNBV=true;
%     explore(handles,useNBV,selection);
else
    msgbox('Not a valid robot pose selected')
end

% --- Executes on button press in find_best_view_pushbutton.
function find_best_view_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
 profile clear;profile on;
% NBV();
NBV_beta();
 profile off;profile viewer;

%publish to the GUi different options
global bestviews;

for cur_view=1:size(bestviews,2)
    if bestviews(cur_view).valid
        bestview_stringcell{cur_view}=...
                strcat('#',num2str(cur_view),': ',num2str(bestviews(cur_view).overall),', is@(',num2str(bestviews(cur_view).scanorigin),...
               '), pose(',num2str(bestviews(cur_view).chosenview),'), Valid(',num2str(bestviews(cur_view).valid),'), addinfoweight=(',num2str(bestviews(cur_view).addinfoweight),'), JointMoveWeight=(',num2str(bestviews(cur_view).jointmoveweight),'), Q=[',num2str(bestviews(cur_view).Q),']');
     end
end

set(handles.best_NBV_listbox,'string',bestview_stringcell,'Value',1);
set(handles.best_NBV_listbox,'visible','on');
set(handles.descripbestview_text,'visible','on');


% --- Executes on button press in remove_selfscan_pushbutton.
function remove_selfscan_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
global workspace
%remove indexed and normal obsticle points within robot FF since not valid
workspace.indexedobsticles=remove_self_scanning(workspace.indexedobsticles);
workspace.obsticlepoints=remove_self_scanning(workspace.obsticlepoints);

% --- Executes on button press in threeDmedianfilter_pushbutton.
function threeDmedianfilter_pushbutton_Callback(hObject, eventdata, handles)
threeDMedianFilt();


%% Classification
% --- Executes on button press in classify_scan_pushbutton.
function classify_scan_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
global scan 

%this is a definition for the number used for unknown material
UNKNOWNVAL=4;
%case 1 = 'Grey Metal '; case 2 = 'Shiny Metal'; case 3 = 'Cloth/Wood'; OR RED OR WHITE --- JUST CLOTH or WOOD!!! case 4 = 'Do not know';

%if NOT USING real robot
if ~get(handles.useRealRobot_checkbox,'Value')
    display('Inputing data for classification from file');
    %load matlab2-from0through-60.mat
    load matlab3-from-60through60.mat
    scan.IntensityData=IntensityData;
    scan.PointData=PointData;
end

%check that PointData exists and is valid
if isfield(scan,'PointData')
    if size(scan.PointData,2)==0
      error('Cant classify since there is no PointData available');
    end
else error('Cant classify since no PointData has been saved to the scan structure - Take a scan with real robot');    
end

%check that IntensityData exists and is valid
if isfield(scan,'IntensityData')
    if size(scan.IntensityData,2)==0
      error('Cant classify since there is no IntensityData available');
    end
else error('Cant classify since no IntensityData has been saved to the scan structure - Take a scan with real robot');    
end

%do classification with data
display('Classifying the LATEST set of data that has been scanned');
scan.ClassificationData=[];
for current_scan=1:size(scan.PointData,1)
    %call Nathan's classifier
    tempval= Classifier(scan.PointData, scan.IntensityData, current_scan);
    %find the valid readings from returned data
    valid=find(tempval.line_start_end_points_smoothed(:,1)>0 & tempval.line_start_end_points_smoothed(:,2)>0);
    %sort out the readings for ease of use (in ascending order)
    sortedranges_withClass=sort([tempval.line_start_end_points_smoothed(valid,:),tempval.classifier_output(valid)]);
    %set all points intially to be unknown
    tempClassData=[squeeze(scan.PointData(current_scan,:,:)),UNKNOWNVAL*ones(size(scan.PointData,2),1)];
    %go through each one of the ranges that have been classfied
    for current_group=1:size(sortedranges_withClass,1)
        %if not UNKNOWNVAL then change to be correct classificaction
        if sortedranges_withClass(current_group,3)~=UNKNOWNVAL
            tempClassData(sortedranges_withClass(current_group,1):sortedranges_withClass(current_group,2),4)=sortedranges_withClass(current_group,3);
        end
    end
    
    %add the classified points to the classifcationdata var
    scan.ClassificationData=[scan.ClassificationData;tempClassData];    
end
display('....Classification completed successfully');


% --- Executes on button press in plot_classified_pushbutton.
function plot_classified_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
global scan

if ~isfield(scan,'ClassificationData') || size(scan.ClassificationData,1)==0
    error('Classification has not happened correctly');
end
%case 1 = 'Grey Metal '; case 2 = 'Shiny Metal'; case 3 = 'Cloth/Wood'; % OR RED OR WHITE --- JUST CLOTH or WOOD!!! case 4 = 'Do not know';

maxclasval=max(scan.ClassificationData(:,4))

%Go through and plot the different data
figure(2);hold on;axis equal;
%set(gcf,'CurrentAxes',handles.axes3);
for current_mat=1:maxclasval
    temp=find(scan.ClassificationData(:,4)==current_mat);
    if ~isempty(temp)
        switch(current_mat)
            case (1) 
                plotcolor=[.6 .6 .6];
            case (2) 
                plotcolor=[.4 .4 .4];
            case (3) 
                plotcolor=[1 0 0];
            case (4) 
                plotcolor=[0.8 0.8 1];
        end        
        if current_mat~=5 plot3(scan.ClassificationData(temp,1),scan.ClassificationData(temp,2),scan.ClassificationData(temp,3),'Marker','.','Color',[plotcolor],'LineStyle','none');end;
    end
end

ClassifiedData=[];
colsiz=size(scan.ClassificationData,1)/size(scan.PointData,1);

for current_row=1:size(scan.PointData,1)
    ClassifiedData=[ClassifiedData;(scan.ClassificationData((current_row-1)*colsiz+1:current_row*colsiz,4)/maxclasval)'];
end

%figure, imshow(ClassifiedData);

%Median filtering
I2=medfilt2(ClassifiedData,[14 14]);
figure, imshow(I2);

Iedges=edge(I2,'canny');
%figure, imshow(Iedges);

[row,col]=ginput(1);  
while col>0 && col<size(Iedges,1) && row>0 && row<size(Iedges,2)        
    if col>0 && col<size(Iedges,1) && row>0 && row<size(Iedges,2)
        row=round(row)-1+find(Iedges(round(col),round(row):end)>0,1);
        boundary=bwtraceboundary(Iedges,[round(col), round(row)],'N');
        hold on;
        plot(boundary(:,2),boundary(:,1),'r','LineWidth',3);
    end
    [row,col]=ginput(1);
end


%% Robot Functionality
% --- Executes on button press in blasting_pushbutton.
function blasting_pushbutton_Callback(hObject, eventdata, handles)
uiwait(msgbox('this does nothing yet'));

% --- Executes on button press in scan_through_pushbutton.
function scan_through_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
global r Q guiglobal
if get(handles.useRealRobot_checkbox,'Value')==0
    doscan();
else
    deg2scan=str2double(get(handles.scan_through_deg_edit,'String'));
    use_real_robot_SCAN(deg2scan);
    organise_data(deg2scan*pi/180);
    use_real_robot_GETJs();
    plotdenso(r, Q, guiglobal.checkFF, guiglobal.plot_ellipse);
end

% --- Executes on button press in move_to_pushbutton.
function move_to_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
%will move to newQ from the GUI
movetonewQ(handles);

% --- Executes on button press in get_current_Js_pushbutton.
function get_current_Js_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
global Q;
if get(handles.useRealRobot_checkbox,'Value'); use_real_robot_GETJs();end
actual_Q=Q*180/pi;
display(strcat('the current joint state of the real robot is:',num2str(actual_Q)));
set(handles.move_to_J1_edit,'String',num2str(actual_Q(1)));
set(handles.move_to_J2_edit,'String',num2str(actual_Q(2)));
set(handles.move_to_J3_edit,'String',num2str(actual_Q(3)));
set(handles.move_to_J4_edit,'String',num2str(actual_Q(4)));
set(handles.move_to_J5_edit,'String',num2str(actual_Q(5)));
set(handles.move_to_J6_edit,'String',num2str(actual_Q(6)));

% --- Executes on button press in alloff_pushbutton.
%turns off all the stuff on the platform
function alloff_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
allOff()

% --- Executes on button press in move2start_platform_pushbutton.
function move2start_platform_pushbutton_Callback(hObject, eventdata, handles)
uiwait(msgbox('this does nothing yet'));

% --- Executes on button press in move2end_platform_pushbutton.
function move2end_platform_pushbutton_Callback(hObject, eventdata, handles)
uiwait(msgbox('this does nothing yet'));

% --- Executes on button press in moveback_platform_pushbutton.
function moveback_platform_pushbutton_Callback(hObject, eventdata, handles)
uiwait(msgbox('this does nothing yet'));

% --- Executes on button press in moveforward_platform_pushbutton.
function moveforward_platform_pushbutton_Callback(hObject, eventdata, handles)
uiwait(msgbox('this does nothing yet'));


%% Plotting and display simple functions
% --- Executes on button press in plotmesh_pushbutton.
function plotmesh_pushbutton_Callback(hObject, eventdata, handles)
global r Q robmap_h
figure(2)
aabb = [-2, -1, 0; 2, 0.6, 2];
%aabb = [-20, -10, -20; 20, 60, 20];
hMesh = robmap_h.Mesh(aabb);
f = hMesh.FaceData;
v = hMesh.VertexData;
trisurf(f, v(:,1), v(:,2), v(:,3), 'FaceColor', 'None');
hold on;
plotdenso(r,Q);
% set(gcf,'CurrentAxes',handles.axes3);

% --- Executes on button press in plot_planes_checkbox.
function plot_planes_checkbox_Callback(hObject, eventdata, handles)%#ok<DEFNU>
global plane guiglobal 
temp_mew=str2double(get(handles.temp_mew_edit,'String'));

theval=get(hObject,'Value');
if theval && ~isempty(plane)
     temp_plane_plot_handles=plot_planes(plane,temp_mew);
     if isfield(guiglobal,'plane_plot_handles')     
         guiglobal.plane_plot_handles=[guiglobal.plane_plot_handles;temp_plane_plot_handles];
     else
         guiglobal.plane_plot_handles=temp_plane_plot_handles;
     end
else
    for current_plane=1:size(guiglobal.plane_plot_handles,1); try delete(guiglobal.plane_plot_handles(current_plane)); end; end;
    guiglobal=rmfield(guiglobal, 'plane_plot_handles');
end

% --- Executes on button press in make_surface_pushbutton.
function make_surface_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
global workspace
temp_mew=str2double(get(handles.temp_mew_edit,'String'));
if size(workspace.obsticlepoints)>0
    set(handles.dialog_text,'String','Making planes....');drawnow;
    surface_making_simple(workspace.obsticlepoints,temp_mew);
    set(handles.dialog_text,'String','successfully made planes');
else
    set(handles.dialog_text,'String','There are no points to make surfaces out of');
end

% --- Executes on button press in rotate_pushbutton.
function rotate_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
rotate3d;

% --- Executes on button press in ResetView.
function ResetView_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
view(2);
axis equal

% --- Executes on button press in sideon_pushbutton.
function sideon_pushbutton_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
axis equal;
view(3);

% --- Executes on button press in plotexplored_pushbutton.
function plotexplored_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
global workspace guiglobal
%can't plot before moving the robot once
if size(workspace.knowncoords,1)==0
   error('not valid when there are no explored (known) points'); 
else
    hold on;
    set(gcf,'CurrentAxes',handles.axes3);
    if ~isfield(guiglobal,'exploredplot')
        guiglobal.exploredplot=plot3(workspace.knowncoords(:,1),workspace.knowncoords(:,2),workspace.knowncoords(:,3),'r.');
        axis vis3d;grid on
    else
        try delete(guiglobal.exploredplot)
        catch display('cant delete the exploration plot');
        end
        guiglobal=rmfield(guiglobal,'exploredplot');
    end
end

% --- Executes on button press in plot_obstacles_pushbutton.
function plot_obstacles_pushbutton_Callback(hObject, eventdata, handles) %#ok<DEFNU>
global  workspace guiglobal
if size(workspace.obsticlepoints,1)==0
   error('not valid when there are no obsticle points'); 
else
    set(gcf,'CurrentAxes',handles.axes3)
    hold on;
    if ~isfield(guiglobal,'obsticleplot')
        %guiglobal.obsticleplot=plot3(workspace.obsticlepoints(:,1),workspace.obsticlepoints(:,2),workspace.obsticlepoints(:,3),'g.');
        guiglobal.obsticleplot=plot3(workspace.indexedobsticles(:,1),workspace.indexedobsticles(:,2),workspace.indexedobsticles(:,3),'g.');
    else
        try delete(guiglobal.obsticleplot);
        catch display('cant delete the obsticle plot');
        end
        guiglobal=rmfield(guiglobal,'obsticleplot');
    end
end

% --- Executes on button press in show_unknownpoints_checkbox.
function show_unknownpoints_checkbox_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
global guiglobal workspace
if ~get(hObject,'value')
    if isfield (guiglobal,'unknownplot')
        for i=1:size(guiglobal.unknownplot,2)
            try delete(guiglobal.unknownplot(i))
            catch display('Cant delete unknown points plot')
            end
        end
        guiglobal=rmfield(guiglobal,'unknownplot');
    end
else
    a=round(   workspace.unknowncoords(workspace.lev1unknown   ,:)/workspace.inc_size);
    b=round(     workspace.knowncoords(GetImpLevInfo(workspace.knowncoords)     ,:)/workspace.inc_size);
    c=round(workspace.indexedobsticles(GetImpLevInfo(workspace.indexedobsticles),:)/workspace.inc_size);
    [nothing,index]=setdiff(a,[b;c],'rows');
    
    if ~isempty(index>0)
        guiglobal.unknownplot(1)=plot3(workspace.unknowncoords(workspace.lev1unknown(index),1),...
                                   workspace.unknowncoords(workspace.lev1unknown(index),2),...
                                   workspace.unknowncoords(workspace.lev1unknown(index),3),...
                                   '.','Color',[1-workspace.dotweight(1) 1-workspace.dotweight(1) 1]);
    else
        display('No Unknown coordinates to plot');
    end
    
end

% --- Executes on button press in show_ellipses_checkbox.
function show_ellipses_checkbox_Callback(hObject, eventdata, handles) %#ok<DEFNU>
global r Q guiglobal densoobj
theval=get(hObject,'Value');
plotrob=get(handles.show_robot_checkbox,'Value');

if theval
    guiglobal.plot_ellipse=1;
    plotdenso(r, Q, guiglobal.checkFF, guiglobal.plot_ellipse);
else
    guiglobal.plot_ellipse=0;
    plotdenso(r, Q, guiglobal.checkFF, guiglobal.plot_ellipse);
end
%deletes the robot if it is not needed
if ~plotrob try for piece=1:size(densoobj,2); delete(densoobj(piece).patches); end; end; end;
    
% --- Executes on button press in show_robot_checkbox.
function show_robot_checkbox_Callback(hObject, eventdata, handles) %#ok<DEFNU>
global r Q guiglobal densoobj
theval=get(hObject,'Value');
if theval
    plotdenso(r, Q, guiglobal.checkFF, guiglobal.plot_ellipse);
else
    try for piece=1:size(densoobj,2); delete(densoobj(piece).patches); end; end
end

% --- Executes on button press in zoom_pushbutton.
function zoom_pushbutton_Callback(hObject, eventdata, handles)
zoom on;

%this function is used to collect the data for exploration to compare new info
function state_data=collectdata(state_data)
global workspace
if isempty(state_data)
    state_data.knownweight=calknownweight();
    state_data.unknownweight=calunknownweight();   
    state_data.size_known=size(workspace.knowncoords,1);
    state_data.size_unknown=size(setdiff(round(workspace.unknowncoords/workspace.inc_size),round(workspace.knowncoords/workspace.inc_size),'rows'),1);
    state_data.size_indexedobsticles=size(workspace.indexedobsticles,1);
    state_data.time=0;state_data.starttime=clock;
else
    state_data.knownweight=[state_data.knownweight,calknownweight()];
    state_data.unknownweight=[state_data.unknownweight,calunknownweight()];
    state_data.size_known=[state_data.size_known,size(workspace.knowncoords,1)];
    state_data.size_unknown=[state_data.size_unknown,size(setdiff(round(workspace.unknowncoords/workspace.inc_size),round(workspace.knowncoords/workspace.inc_size),'rows'),1)];
    state_data.size_indexedobsticles=[state_data.size_indexedobsticles,size(workspace.indexedobsticles,1)];
    state_data.time=[state_data.time,etime(clock,state_data.starttime)];
end    



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Not Used By Me
% --- Executes during object creation, after setting all properties.
function best_NBV_listbox_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

% --- Executes on button press in useRealRobot_checkbox.
function useRealRobot_checkbox_Callback(hObject, eventdata, handles)

function scan_through_deg_edit_Callback(hObject, eventdata, handles)
% --- Executes during object creation, after setting all properties.
function scan_through_deg_edit_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Joint 1 edit create and callback
function move_to_J1_edit_Callback(hObject, eventdata, handles)
function move_to_J1_edit_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Joint 2 edit create and callback
function move_to_J2_edit_Callback(hObject, eventdata, handles)
function move_to_J2_edit_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%--- Joint 3 edit create and callback
function move_to_J3_edit_Callback(hObject, eventdata, handles)
function move_to_J3_edit_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Joint 4 edit create and callback
function move_to_J4_edit_Callback(hObject, eventdata, handles)
function move_to_J4_edit_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Joint 5 edit create and callback
function move_to_J5_edit_Callback(hObject, eventdata, handles)
function move_to_J5_edit_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Joint 6 edit create and callback
function move_to_J6_edit_Callback(hObject, eventdata, handles)
function move_to_J6_edit_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in start_with_default_poses_checkbox.
function start_with_default_poses_checkbox_Callback(hObject, eventdata, handles)

% --- Executes on button press in animate_move_checkbox.
function animate_move_checkbox_Callback(hObject, eventdata, handles)

% --- Executes on button press in exact_joints_only_checkbox.
function exact_joints_only_checkbox_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function temp_mew_edit_Callback(hObject, eventdata, handles)
function temp_mew_edit_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_2_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_3_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_4_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_5_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


