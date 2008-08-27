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
% global Q vid_obect
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

%% to stop do this
% stop(vid)
% flushdata(vid)
% delete(vid)

%% other useful functions
 
% get(vid)
% get(getselectedsource(vid))
% set(getselectedsource(vid),'FrameRate','10')
 
%  preview(vid);
% closepreview(vid)









% for i=1:numoftrigs
%     frametowrite=frame(i).val;
%     imwrite(frametowrite,['images\',fr
      