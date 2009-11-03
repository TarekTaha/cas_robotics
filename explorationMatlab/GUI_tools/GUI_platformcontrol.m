function GUI_platformcontrol(command)
if nargin==0
    error('need to pass a command');
end
if strcmp(command,'allOff')
    allOff();
    
elseif strcmp(command,'MoveToHome')
    hCOM=getappdata(gcf,'hCOM');
    platform_h=hCOM.PlatformCommand;
    platform_h.Type = 'MoveToHome';
    platform_h.Start;
    
elseif strcmp(command,'MoveToEnd')    
    hCOM=getappdata(gcf,'hCOM');
    platform_h=hCOM.PlatformCommand;
    platform_h.Type = 'MoveToEnd';
    platform_h.Start;
    
elseif strcmp(command,'MoveBackward')
    hCOM=getappdata(gcf,'hCOM');
    platform_h=hCOM.PlatformCommand;
    platform_h.Type = 'MoveBackward';
    platform_h.Start;
    
elseif strcmp(command,'MoveForward')
    hCOM=getappdata(gcf,'hCOM');
    platform_h=hCOM.PlatformCommand;
    platform_h.Type = 'MoveForward';
    platform_h.Start;
else 
    error('Incorrect command passed into GUI_platformcontrol');
end