%% setupvideo
%
% *Description:* This function sets up the variables of the video if
% possible

%% Function Call
% 
% *Inputs:* NULL
%
% *Returns:* NULL

function setupvideo()

%% Variablesj
global vid_object 

try 
%% Variables
vid_object.imagefileEXT='png';

% M2M02info= imaqhwinfo('winvideo');
% all_formats=M2M02info.DeviceInfo.SupportedFormats;
%     'I420_160x120'    'I420_176x144'    'I420_320x240'    'I420_352x288'
%     'I420_640x480' 'RGB24_160x120'    'RGB24_176x144'    'RGB24_320x240'
%     'RGB24_352x288'    'RGB24_640x480' 
vid_object.ourformat='RGB24_640x480';

% reset and delete all previois video objects and data
imaqreset

vid_object.vid = videoinput('winvideo',1,vid_object.ourformat);

%% setup triggers
set(vid_object.vid,'FramesPerTrigger',1);
triggerconfig(vid_object.vid,'manual')
set(vid_object.vid,'TriggerRepeat',inf);
start(vid_object.vid)
catch
  vid_object=[];
  display('No camera found on winvideo');
end
  

%% To trigger and register arm pose do this run
% global Q vid_object
% numoftrigs=100;
% for i=1:numoftrigs 
%   display(['The video running state',num2str(isrunning(vid_object.vid))]);
%   trigger(vid_object.vid)
%   frame(i).val=getdata(vid_object.vid);imshow(frame(i).val)
%   frame(i).filename=[datestr(now,'yyyy-mm-dd HHMMSS'),' POSE ',num2str(Q)];
% end
% for i=1:numoftrigs
%     frametowrite=frame(i).val;
%     imwrite(frametowrite,['images\',frame(i).filename,'.',imagefileEXT],imagefileEXT);
% end

% for i=1:numoftrigs
%     frametowrite=frame(i).val;
%     imwrite(frametowrite,['images\',fr
% end

%% to stop do this
% stop(vid_object.vid)
% flushdata(vid_object.vid)
% delete(vid_object.vid)

%% other useful functions
 
% get(vid_object.vid)
% get(getselectedsource(vid_object.vid))
% set(getselectedsource(vid_object.vid),'FrameRate','10')
 
%  preview(vid_object.vid);
% closepreview(vid_object.vid)


%calibration with an A1 (594 × 841) sheet in front resting on pront strut
% -1.88         23.1         99.2         7.13        28.95        -3.43
%makes for a distance of about a meter (1000mm) to each corner
%for 6DOF robot
% fkine(r,Q)
% 
% ans =
% 
%    -0.8734    0.0467    0.4847    0.8038
%     0.0777    0.9960    0.0442   -0.0210
%    -0.4807    0.0763   -0.8736    0.5973
%          0         0         0    1.0000
%BUT FOR A 7DOF robot, absolutely beautiful
% fkine(r,[Q])
% ans =
%    -0.0000    0.7071    0.7071    0.5820
%     1.0000    0.0000    0.0000    0.0000
%     0.0000    0.7071   -0.7071    0.5548
%          0         0         0    1.0000
         
%FOV in horizontal
% rad2deg(acos((2*1000^2-841^2)/(2*1000^2)))
% =49.7323
% approx 50'

%FOV in vertical
% rad2deg(acos((2*1000^2-594^2)/(2*1000^2)))
% =34.5550
%approx 35'


      