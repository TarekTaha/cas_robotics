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

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
reply = input('Do you want to delete the mesh from memory and release object 1/0 [1]: ');
if isempty(reply)
    reply = 1;
end
if reply
    global robmap_h;
    try robmap_h.release;end
end
%try and release platform object
global platform_h;
try platform_h.release;end

delete(hObject);


%% Executes on button press in clear_pushbutton.
function clear_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
clc
do_clear(handles);

% this is the do_clear function sets up most stuff
function do_clear(handles)

%add path 
% keyboard
% addpath('./EnvironmentalData')

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
pause


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
    display('EyeInHand Problem: Unable to create surface map')
end

% try and setup platform object
try global platform_h;
    try platform_h.release;pause(1);end
    platform_h = actxserver('EyeInHand.PlatformCommand');
catch
    display('EyeInHand Problem: Unable to create platform object')
end


%clear the main axis
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

%clear the globals for scan, workspace, robot(r,Q), bestviews, PointData, RangeData
clear global workspace scan bestviews Q r PointData RangeData guiglobal densoobj all_views robot_maxreach classunkn_optimise alldirectedpoints graf_obs;

%Sets up the robot
profile clear;profile on;
setuprobot()
profile off;profile viewer;

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


global all_views
if isempty(all_views)
    warning('not calculating allviews even though it dosent exist');
%     display('Having to calculate all_views for exploration, this happens ones only');
%     calc_all_views();
%     load all_views.mat
end

clc;

set(handles.dialog_text,'String','Setup Complete: Lets Explore');


%% Exploration Functionality
% --- Executes on button press in stopflag_checkbox.
function stopflag_checkbox_Callback(hObject, eventdata, handles)
set(handles.stopflag_checkbox,'Visible','off')
% hObject    handle to stopflag_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of stopflag_checkbox


% --- Executes on button press in go_pushbutton.
function go_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
set(handles.stopflag_checkbox,'Value',0)
set(handles.stopflag_checkbox,'Visible','on')

%%%CHOOSE THE CURRENT TEST CASE
current_test_case=2; %1 = random, 2 = Next best view search
show_new_info_details=false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% profile clear; profile on;
global robot_maxreach optimise workspace scan Q

%update Q to the latest actual value of the robot
if get(handles.useRealRobot_checkbox,'Value')==1;  use_real_robot_GETJs();end   

%start data collection
state_data=collectdata([]);
useNBV=false;
set(gcf,'CurrentAxes',handles.axes3)

%% intial safe pose default scans
%since we wish to choose our own default start points
for stepcount=2:size(robot_maxreach.default_Q,1)+1
%     want_to_continue=true;
    want_to_continue=~get(handles.stopflag_checkbox,'Value');
    while want_to_continue && scan.tries<size(robot_maxreach.default_Q,1)
        try explore(handles,useNBV); 
            want_to_continue=~get(handles.stopflag_checkbox,'Value');
            if want_to_continue==0; error('User chose to exit');end
            break;             
        catch
            display(lasterr);
            want_to_continue=input('Type (1) to continue, (2) for keyboard command, (0) to exit\n');            
            if want_to_continue==2; keyboard; end
            if want_to_continue==0; error('User chose to exit');end
            if get(handles.useRealRobot_checkbox,'Value')==1;  
                use_real_robot_GETJs();
                set(handles.stopflag_checkbox,'Value',0);
                set(handles.stopflag_checkbox,'Visible','on');
            end   
        end; 
    end
    state_data=collectdata(state_data); 
end

want_to_continue=~get(handles.stopflag_checkbox,'Value');

%% TEST 1 - random points
if current_test_case==1 && want_to_continue
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
                [pathfound,all_steps]=pathplanner_new(newQ);
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
elseif current_test_case>1 && want_to_continue
    %now go through and get NBV and then use them to explore
    for stepcount=stepcount+1:15;
        if ~get(handles.stopflag_checkbox,'Value')
            %do next best view exploration search
%             profile clear; profile on;
            NBV_beta2();
%             profile off; profile viewer;

            current_bestview=1;
            max_bestviews_togothrough=optimise.valid_max*1/4;
            global bestviews;        
            while current_bestview<max_bestviews_togothrough && size(bestviews,2)>=1 && ~get(handles.stopflag_checkbox,'Value')
                %size(bestviews,2)>optimise.valid_max*3/4
                current_bestview=current_bestview+1;

                useNBV=true;
    %             want_to_continue=true;
                want_to_continue=~get(handles.stopflag_checkbox,'Value');   
                
                while want_to_continue; 
                    try %if we have already planned a path, use this one otherwise try and get another, otherwise go to next possible one
                        if movetonewQ(handles,bestviews(1).Q*180/pi,bestviews(1).all_steps);
                            scan.done_bestviews_orfailed=[scan.done_bestviews_orfailed;bestviews(1).Q];explore(handles,useNBV,1);validpathfound=true;break;
                        else %can't get to the desired best view
%                             display('User has control');
%                             keyboard
                            
                            %tac on the actual position here just in case
                            %it isn't exactly where it was supposed to
                            %finish
                            robot_maxreach.path(end).all_steps(end+1,:)=Q;
                            %move back along the path taken to get here
                            if ~movetonewQ(handles,robot_maxreach.path(end).all_steps(1,:)*180/pi,robot_maxreach.path(end).all_steps(end:-1:1,:));
                                display('some major problem if we cant follow the same path back');
                                keyboard                                
                            end
                            %try once again to move to the actual desired
                            %newQ for exploration
                            if movetonewQ(handles,bestviews(1).Q*180/pi,bestviews(1).all_steps);
                                scan.done_bestviews_orfailed=[scan.done_bestviews_orfailed;bestviews(1).Q];explore(handles,useNBV,1);validpathfound=true;break;
                            else % last resort is to remove surrounding obstacle points remove indexed and normal obsticle points within robot FF since not valid
                                display(['No Valid path available or found, on #',num2str(current_bestview)]);
                                validpathfound=false;
                            end
                        end
                        
                        want_to_continue=0;                        
                    catch; display(lasterr);
                        want_to_continue=input(' Type (0) to go to new best view, (1) to continue trying for a path, (2) for keyboard command, (3) to exit\n');            
                        if want_to_continue==2; keyboard; end
                        if want_to_continue==3; error('User chose to exit');end
                        if get(handles.useRealRobot_checkbox,'Value')==1;  use_real_robot_GETJs();end 
                        display(['No Valid path available or found, on #',num2str(current_bestview)]);
                        validpathfound=false;
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

                %termination conditions
                changeinweight=diff(state_data.knownweight);
                if length(changeinweight)>3
                    if sum(changeinweight(end-3:end))<100
                        %set to stop ASAP
                        set(handles.stopflag_checkbox,'Value',1);
                        display('Termination condition reached');
                        break;
                    end
                end
                    
                if size(bestviews,2)>1  
                    %Resize bestviews so as to get rid of the first element
                    clear temp_bestviews;
                    for cur_view=2:size(bestviews,2); temp_bestviews(cur_view-1)=bestviews(cur_view);end
                        bestviews=temp_bestviews;
                        %if we actually found a place to move and explore
                        %from record new data
                        if validpathfound==true
                            state_data=collectdata(state_data); 
                        end

                        redo_nbv_vol=true;
                        order_bestviews(redo_nbv_vol);
                else
                    bestviews=[];
                end
            end
        end
    end
end


%Plotting the results
figure;
subplot(3,1,1);plot(state_data.knownweight);grid on; title('Known Weight');
subplot(3,1,2);plot(state_data.size_known);grid on; title('Total Known Points');
subplot(3,1,3);plot(state_data.time);grid on; title('Time Taken');

% Saving for journal results
% save('state_data.mat','state_data');
% testnum=input('test num');
% saveresultstofile(testnum);

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
    pointAT=bestviews(selection).chosenview(:)+bestviews(selection).scanorigin(:);
    tempplothandle= [tempplothandle,plot3([bestviews(selection).scanorigin(1),pointAT(1)],[bestviews(selection).scanorigin(2),pointAT(2)],[bestviews(selection).scanorigin(3),pointAT(3)])];   
    uiwait(msgbox('have a look then press ok'))
    try for i=1:length(tempplothandle); delete(tempplothandle(i)); end; end;
%plot the new info    
    tempplothandle= plot3(bestviews(selection).expectedaddinfo(:,1),bestviews(selection).expectedaddinfo(:,2),bestviews(selection).expectedaddinfo(:,3),'y.');
    uiwait(msgbox('have a look then press ok'))
    try delete(tempplothandle); end;
%%%%%%%%ENDdelete this plotting stuff    %%%%%%%%%%%
    
    useNBV=true;
    display('NOT setup to explore from this view');
%     explore(handles,useNBV,selection);
else
    msgbox('Not a valid robot pose selected')
end

% --- Executes on button press in find_best_view_pushbutton.
function find_best_view_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
%  profile clear;profile on;
NBV_beta2();
%  profile off;profile viewer;

%publish to the GUi different options
global bestviews;

for cur_view=1:size(bestviews,2)
    if bestviews(cur_view).valid
        bestview_stringcell{cur_view}=...
                strcat('#',num2str(cur_view),': ',num2str(bestviews(cur_view).overall),', is@(',num2str(bestviews(cur_view).scanorigin(:)'),...
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
% workspace.Nobsticlepoints=remove_self_scanning(workspace.Nobsticlepoints);

% --- Executes on button press in threeDmedianfilter_pushbutton.
function threeDmedianfilter_pushbutton_Callback(hObject, eventdata, handles)
threeDMedianFilt();


%% Classification
% --- Executes on button press in classify_scan_pushbutton.
function classify_scan_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
global scan PointData IntensityData RangeData workspace
%case 1 = 'Grey Metal '; case 2 = 'Shiny Metal'; case 3 = 'Cloth/Wood'; OR RED OR WHITE --- JUST CLOTH or WOOD!!! case 4 = 'Do not know';

%if NOT USING real robot load data
if ~get(handles.useRealRobot_checkbox,'Value')
    display('Inputing data for classification from file');
    load ScanforClassifier-0to-60.mat
    scan.IntensityData=IntensityData;
    scan.PointData=PointData;
    scan.RangeData=RangeData;
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

%check that PointData exists and is valid
if isfield(scan,'RangeData')
    if size(scan.RangeData,2)==0
      error('Cant classify since there is no RangeData available');
    end
else error('Cant classify since no RangeData has been saved to the scan structure - Take a scan with real robot');    
end

%do classification with data
display('Classifying the LATEST set of data that has been scanned');
scan.ClassificationData=[];

try [ClassifiedData] = Block_Classifier(PointData, IntensityData,RangeData); 
% profile clear;profile on;    
try UNclassifiedvoxels=update_ocstatus(ClassifiedData); %profile off; profile viewer;
        display(['You still have not classified ', num2str(size(UNclassifiedvoxels,1)/size(workspace.ocgrid,1)*100),' percent of known voxels']);
        display('....Classification completed successfully');    
    catch; display('Couldnt update voxels'); keyboard; end        
catch; display('Couldnt classify');keyboard; end
    
    

% --- Executes on button press in AXBAMnClassify_pushbutton.
function AXBAMnClassify_pushbutton_Callback(hObject, eventdata, handles)
%num of planes to tryand get to of interest
numofintplanes=str2double(get(handles.numofdir_AXBAMnC_edit,'string'));
%run the classification function    
poseclassunknown_Imp(numofintplanes)



% --- Executes on button press in plot_classified_pushbutton.
% function plot_classified_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
% global scan
% 
% if ~isfield(scan,'ClassificationData') || size(scan.ClassificationData,1)==0
%     error('Classification has not happened correctly');
% end
% %case 1 = 'Grey Metal '; case 2 = 'Shiny Metal'; case 3 = 'Cloth/Wood'; % OR RED OR WHITE --- JUST CLOTH or WOOD!!! case 4 = 'Do not know';
% 
% maxclasval=max(scan.ClassificationData(:,4))
% 
% %Go through and plot the different data
% figure(2);hold on;axis equal;
% %set(gcf,'CurrentAxes',handles.axes3);
% for current_mat=1:maxclasval
%     temp=find(scan.ClassificationData(:,4)==current_mat);
%     if ~isempty(temp)
%         switch(current_mat)
%             case (1) 
%                 plotcolor=[.6 .6 .6];
%             case (2) 
%                 plotcolor=[.4 .4 .4];
%             case (3) 
%                 plotcolor=[1 0 0];
%             case (4) 
%                 plotcolor=[0.8 0.8 1];
%         end        
%         if current_mat~=5 plot3(scan.ClassificationData(temp,1),scan.ClassificationData(temp,2),scan.ClassificationData(temp,3),'Marker','.','Color',[plotcolor],'LineStyle','none');end;
%     end
% end
% 
% ClassifiedData=[];
% colsiz=size(scan.ClassificationData,1)/size(scan.PointData,1);
% 
% for current_row=1:size(scan.PointData,1)
%     ClassifiedData=[ClassifiedData;(scan.ClassificationData((current_row-1)*colsiz+1:current_row*colsiz,4)/maxclasval)'];
% end
% 
% %figure, imshow(ClassifiedData);
% 
% %Median filtering
% I2=medfilt2(ClassifiedData,[14 14]);
% figure, imshow(I2);
% 
% Iedges=edge(I2,'canny');
% %figure, imshow(Iedges);
% 
% [row,col]=ginput(1);  
% while col>0 && col<size(Iedges,1) && row>0 && row<size(Iedges,2)        
%     if col>0 && col<size(Iedges,1) && row>0 && row<size(Iedges,2)
%         row=round(row)-1+find(Iedges(round(col),round(row):end)>0,1);
%         boundary=bwtraceboundary(Iedges,[round(col), round(row)],'N');
%         hold on;
%         plot(boundary(:,2),boundary(:,1),'r','LineWidth',3);
%     end
%     [row,col]=ginput(1);
% end


%% Robot Functionality
% --- Executes on button press in blasting_pushbutton.
function blasting_pushbutton_Callback(hObject, eventdata, handles)
keyboard
return

global robmap_h
figure(2)
view(2)

try 
    if exist('verts.mat','file')==2 
        load verts.mat;
        surface_making_simple(verts,0.04)
        global plane
        
        pathPlannerV4(handles);
    else
        rect_vars=getrect;
        min_max=[rect_vars(1),rect_vars(2),1.4;
        rect_vars(1)+rect_vars(3),rect_vars(2)+rect_vars(4),1.6];

        hMesh = robmap_h.Mesh(min_max);
        verts = hMesh.VertexData;
        save verts

        surface_making_simple(verts,0.04)
        global plane

        if size(plane,2)>30
            plane=plane(1:30);
        end
        save plane
    end
catch
    error('failed to get verts or create targets');
end

%this will be andrew's function
cd RTA
setupoptimisation
keyboard
PathplannerV5(handles)


% --- Executes on button press in tilt_scan_pushbutton.
function tilt_scan_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
global r Q guiglobal
if get(handles.useRealRobot_checkbox,'Value')==0
    doscan();
else
    deg2scan=str2double(get(handles.tilt_scan_deg_edit,'String'));
    use_real_robot_SCAN(deg2scan);
    organise_data();
    use_real_robot_GETJs();
    plotdenso(r, Q, guiglobal.checkFF, guiglobal.plot_ellipse);
end

% --- Executes on button press in rot_scan_pushbutton.
function rot_scan_pushbutton_Callback(hObject, eventdata, handles) %#ok<DEFNU>
global r Q guiglobal
if get(handles.useRealRobot_checkbox,'Value')==0
    error('Cant do this type of scan yet');
else
    deg2scan=str2double(get(handles.rot_scan_deg_edit,'String'));
    use_real_robot_SCANandMOVE(deg2scan);
    organise_data()
    use_real_robot_GETJs();
    plotdenso(r, Q, guiglobal.checkFF, guiglobal.plot_ellipse);
end


% --- Executes on button press in move_to_pushbutton.
function move_to_pushbutton_Callback(hObject, eventdata, handles)%#ok<DEFNU>
%will move to newQ from the GUI
% profile clear; profile on;
movetonewQ(handles);
% profile off; profile viewer;


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
global platform_h
if isempty(platform_h)
    platform_h = actxserver('EyeInHand.PlatformCommand');
end
platform_h.Type = 'MoveToHome';
platform_h.Start;


% --- Executes on button press in move2end_platform_pushbutton.
function move2end_platform_pushbutton_Callback(hObject, eventdata, handles)
global platform_h
if isempty(platform_h)
    platform_h = actxserver('EyeInHand.PlatformCommand');
end
platform_h.Type = 'MoveToEnd';
platform_h.Start;


% --- Executes on button press in moveback_platform_pushbutton.
function moveback_platform_pushbutton_Callback(hObject, eventdata, handles)
global platform_h
if isempty(platform_h)
    platform_h = actxserver('EyeInHand.PlatformCommand');
end
platform_h.Type = 'MoveBackward';
platform_h.Start;


% --- Executes on button press in moveforward_platform_pushbutton.
function moveforward_platform_pushbutton_Callback(hObject, eventdata, handles)
global platform_h
if isempty(platform_h)
    platform_h = actxserver('EyeInHand.PlatformCommand');
end
platform_h.Type = 'MoveForward';
platform_h.Start;
%     platform_h.WaitUntilCompleted(20,0);
% allOff()


% platform_h.Type = 'NoMotion';
% platform_h.Type = 'MoveForward';
% platform_h.Type = 'MoveBackward';
% platform_h.Type = 'MoveToHome';
% platform_h.Type = 'MoveToEnd';


% platform_h = actxserver('EyeInHand.PlatformState');
%  get(platform_h,'Status')==0 && get(platform_h,'EmergencyStop')==0 %%this means OK
%     platform_h.WarningLight = 1; % Turn on
% 
%     % platform_h.WarningNoise = 1;
%     % pause(1)
%     % platform_h.WarningNoise = 0;
% 
%     platform_h.AirSupply = 1; % Turn on
%     platform_h.Brake = 0; % Turn off
%     platform_h.Motors = 'Forward';
%     pause(2.0);
%     platform_h.Motors = 'Off';
%     platform_h.Brake = 1; % Turn on
%     platform_h.WarningLight = 0; % Turn on
%     platform_h.AirSupply = 0; % Turn off
     
% platform_h.release


%% Plotting and display simple functions
% --- Executes on button press in plot_special_checkbox.
function plot_special_checkbox_Callback(hObject, eventdata, handles)
global workspace guiglobal
%can't plot before moving the robot once
if size(workspace.spec_pnts,1)==0
   error('not valid when there are no spec_pnts'); 
else
    hold on;
    set(gcf,'CurrentAxes',handles.axes3);
    if ~isfield(guiglobal,'special_pntsplot')
        guiglobal.special_pntsplot=plot3(workspace.spec_pnts(:,1),workspace.spec_pnts(:,2),workspace.spec_pnts(:,3),'k*');
        axis vis3d;grid on
    else
        try delete(guiglobal.special_pntsplot)
        catch display('cant delete the special points plot');
        end
        guiglobal=rmfield(guiglobal,'special_pntsplot');
    end
end

% --- Executes on button press in plot_arm_carved_checkbox.
function plot_arm_carved_checkbox_Callback(hObject, eventdata, handles)
global robot_maxreach guiglobal
%can't plot before moving the robot once
if size(robot_maxreach.pointcarvedout,1)==0
   error('not valid when there are no points carved out yet'); 
else
    hold on;
    set(gcf,'CurrentAxes',handles.axes3);
    if ~isfield(guiglobal,'pointcarvedout')
        guiglobal.pointcarvedout=plot3(robot_maxreach.pointcarvedout(:,1),robot_maxreach.pointcarvedout(:,2),robot_maxreach.pointcarvedout(:,3),'b.');
        axis vis3d;grid on
    else
        try delete(guiglobal.pointcarvedout)
        catch display('cant delete the pointcarvedout plot');
        end
        guiglobal=rmfield(guiglobal,'pointcarvedout');
    end
end

% --- Executes on button press in plotmesh_pushbutton.
function plotmesh_pushbutton_Callback(hObject, eventdata, handles)
global robmap_h guiglobal

try delete(guiglobal.mesh_h);
catch
    % figure(2)
    set(gcf,'CurrentAxes',handles.axes3);
    % aabb = [-2, -1, 0; 2, 0.6, 2];
    
    aabb = [-1.5, -1.5, -1; 2, 1.5, 2];
%     aabb = [-1.5, 0.3, -1; 2, 0.7, 2];
%     if we want to show the whole environment
    if get(handles.all_mesh_checkbox,'value')==1
        aabb = [-20, -20, -20; 20, 20, 20];
    end
    hMesh = robmap_h.Mesh(aabb);
    f = hMesh.FaceData;
    v = hMesh.VertexData;
    % save('datafile.mat','v');
    hold on;
    guiglobal.mesh_h=trisurf(f, v(:,1), v(:,2), v(:,3), 'FaceColor', 'None');
    axis equal
end


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
global robmap_h 
      
try aabb = [-1.5, -1.5, -1; 2, 1.5, 2];
hMesh = robmap_h.Mesh(aabb);
obsticlepoints = hMesh.VertexData(GetImpLevInfo(hMesh.VertexData),:);

catch; error('In getting the mesh needed for surface making');
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
function ResetView_Callback(hObject, eventdata, handles)%#ok<INUSD,DEFNU>
global guiglobal
try delete(guiglobal.mesh_h);end
set(gcf,'CurrentAxes',handles.axes3);
cla('reset')
camlight
lighting gouraud
axis equal
view(2);
colordef white

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
if size(workspace.indexedobsticles,1)==0
   error('not valid when there are no indexedobsticles points'); 
else
    set(gcf,'CurrentAxes',handles.axes3)
    hold on;
    if ~isfield(guiglobal,'obsticleplot')
        guiglobal.obsticleplot=plot3(workspace.indexedobsticles(:,1),workspace.indexedobsticles(:,2),workspace.indexedobsticles(:,3),'g.');
    else
        try delete(guiglobal.obsticleplot);
        catch display('cant delete the obsticle plot');
        end
        guiglobal=rmfield(guiglobal,'obsticleplot');
    end
end

% --- Executes on button press in plot_classified_checkbox.
function plot_classified_checkbox_Callback(hObject, eventdata, handles)
global workspace classifiedplotHa

if get(hObject,'Value')
    display('Drawing plot')  
    classifiedplotHa=[];
    class_cubesize=workspace.class_cubesize;
    
    %determine the unknown places and the know metal or wood
    sumofclass=workspace.ocgrid(:,4)+workspace.ocgrid(:,5);
    warning('off','MATLAB:divideByZero')
    try classifiedvoxels=find(sumofclass>=workspace.minclassifications &...
                         (workspace.ocgrid(:,4)./workspace.ocgrid(:,5)>workspace.classfierthreshhold |...
                          workspace.ocgrid(:,5)./workspace.ocgrid(:,4)>workspace.classfierthreshhold));end
    try UNclassifiedvoxels=find(sumofclass<workspace.minclassifications | ...
                         (workspace.ocgrid(:,4)./workspace.ocgrid(:,5)<=workspace.classfierthreshhold &...
                          workspace.ocgrid(:,5)./workspace.ocgrid(:,4)<=workspace.classfierthreshhold));end
    warning('on','MATLAB:divideByZero')
    
    %now plot this
    hold on;
    try classifiedplotHa(end+1)=plot3(workspace.ocgrid(UNclassifiedvoxels,1)*class_cubesize,...
          workspace.ocgrid(UNclassifiedvoxels,2)*class_cubesize,...
          workspace.ocgrid(UNclassifiedvoxels,3)*class_cubesize,'y','marker','.','markersize',0.5,'linestyle','none');end
    %plot metal and wood voxels

    metalvoxels=workspace.ocgrid(classifiedvoxels(workspace.ocgrid(classifiedvoxels,4)>workspace.ocgrid(classifiedvoxels,5)),1:3);
    if size(metalvoxels,1)>0
        try classifiedplotHa(end+1)=plot3(metalvoxels(:,1)*class_cubesize,metalvoxels(:,2)*class_cubesize,metalvoxels(:,3)*class_cubesize,'r.');end
    end

    woodvoxels=workspace.ocgrid(classifiedvoxels(workspace.ocgrid(classifiedvoxels,4)<workspace.ocgrid(classifiedvoxels,5)),1:3);
    if size(woodvoxels,1)>0
        try classifiedplotHa(end+1)=plot3(woodvoxels(:,1)*class_cubesize,woodvoxels(:,2)*class_cubesize,woodvoxels(:,3)*class_cubesize,'b.');end
    end

    drawnow

else
    display('Removing plot')
    for i=1:length(classifiedplotHa);  try delete(classifiedplotHa(i));end; end
    clear global classifiedplotHa
    
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
    DrawEnv(true);
else
    try for piece=1:size(densoobj,2); delete(densoobj(piece).patches); end; end
    DrawEnv(false);
end

% --- Executes on button press in zoom_pushbutton.
function zoom_pushbutton_Callback(hObject, eventdata, handles)
% zoom on;
view(116,18)
keyboard


%this function is used to collect the data for exploration to compare new info
function state_data=collectdata(state_data)
global workspace Q
if isempty(state_data)
    state_data.knownweight=calknownweight();
    state_data.unknownweight=calunknownweight();   
    state_data.size_known=size(workspace.knowncoords,1);
    state_data.size_unknown=size(setdiff(round(workspace.unknowncoords/workspace.inc_size),round(workspace.knowncoords/workspace.inc_size),'rows'),1);
    state_data.size_indexedobsticles=size(workspace.indexedobsticles,1);
    state_data.time=0;state_data.starttime=clock;
    state_data.Q=Q;
else
    state_data.knownweight=[state_data.knownweight,calknownweight()];
    state_data.unknownweight=[state_data.unknownweight,calunknownweight()];
    state_data.size_known=[state_data.size_known,size(workspace.knowncoords,1)];
    state_data.size_unknown=[state_data.size_unknown,size(setdiff(round(workspace.unknowncoords/workspace.inc_size),round(workspace.knowncoords/workspace.inc_size),'rows'),1)];
    state_data.size_indexedobsticles=[state_data.size_indexedobsticles,size(workspace.indexedobsticles,1)];
    state_data.time=[state_data.time,etime(clock,state_data.starttime)];
    state_data.Q=[state_data.Q;Q];
end    

% --- Executes on button press in savefig_pushbutton.
function savefig_pushbutton_Callback(hObject, eventdata, handles)
try 
    AXBAMnCtesting();
catch
    lasterr
    keyboard
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

function tilt_scan_deg_edit_Callback(hObject, eventdata, handles)
% --- Executes during object creation, after setting all properties.
function tilt_scan_deg_edit_CreateFcn(hObject, eventdata, handles)
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

%this is for the edit box of how many planes of interest we are going to go
%through
function AXBAMnC_directs_edit_Callback(hObject, eventdata, handles)
function AXBAMnC_directs_edit_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function numofdir_AXBAMnC_edit_Callback(hObject, eventdata, handles)
function numofdir_AXBAMnC_edit_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function rot_scan_deg_edit_Callback(hObject, eventdata, handles)
% --- Executes during object creation, after setting all properties.
function rot_scan_deg_edit_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in scanwhilemove_checkbox.
function scanwhilemove_checkbox_Callback(hObject, eventdata, handles)

% --- Executes on button press in remv_unkn_in_mv_checkbox.
function remv_unkn_in_mv_checkbox_Callback(hObject, eventdata, handles)

% --- Executes on button press in doclassification_checkbox.
function doclassification_checkbox_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function minClassifications_edit_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function minClassifications_edit_Callback(hObject, eventdata, handles)
global workspace
workspace.minclassifications=str2double(get(hObject,'String'));

% --- Executes during object creation, after setting all properties.
function classification_ration_edit_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function classification_ration_edit_Callback(hObject, eventdata, handles)
global workspace
workspace.classfierthreshhold=str2double(get(hObject,'String'));


%% dont know why this is needed, can't find the button for it
function Untitled_1_Callback(hObject, eventdata, handles)
