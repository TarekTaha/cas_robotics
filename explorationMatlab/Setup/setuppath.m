function setuppath()

% We are now in exGUI\Setup
%get the path out of the mfilename return
pathstr= fileparts(mfilename('fullpath'));

%Add the path directories for exGUI
path([pathstr,'\..\Blasting'],path);
path([pathstr,'\..\Classify'],path);
path([pathstr,'\..\EnvironmentalData'],path);
path([pathstr,'\..\Explore'],path);
path([pathstr,'\..\GUI_tools'],path);
path([pathstr,'\..\Planning'],path);
path([pathstr,'\..\Tools'],path);


%also need to have added to the path something like 
%'\RTA_Project\Projects\0906 Matlab UI\RTA_Proj02'
%Add the coms directory to path
if ~exist('COM_Create_Coms.m','file') 
    if exist([matlabroot,'\work\0906 UI\RTA_Proj02\COM'],'dir')
        addpath([matlabroot,'\work\0906 UI\RTA_Proj02\COM']);
    elseif exist('D:\RTA_Project\Projects\0906 Matlab UI\RTA_Proj02\COM','dir')
        addpath('D:\RTA_Project\Projects\0906 Matlab UI\RTA_Proj02\COM');
    else
      error(['Cant find necessary COM setup components ',...
        'SVN latest from https://develop.eng.uts.edu.au/projects/cas/grit_blasting/svn/trunk/Projects/0906%20Matlab%20UI/RTA_Proj02 ',...
        'Then add to path or at least place in: matlabroot,\work\0906 UI\RTA_Proj02']);
    end
end