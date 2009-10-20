function setupexplorationmain(handles)

% Check if we have a GUI 
if nargin==0
    DOsetupGUI=false;
    showunknownpnts=false;
else
    DOsetupGUI=true;
    showunknownpnts=get(handles.show_unknownpoints_checkbox,'Value');
end


try
    %set the paths used in exGUI
    setuppath()
    
    %setup the GUI
    if DOsetupGUI
        setupGUI(handles);
    end

    %Sets up the robot
    setuprobot();

    % Sets up the scanner
    setupscanner();

    %this sets up the joint optimisation and NBV optimisation 
    setupoptimisation();
    
    setupworkspace(showunknownpnts);

    % Create COMS (must be after the workspace is setup)
    setupCOMs(gcf);
    
    
    
    

catch  %#ok<CTCH>
    lasterr %#ok<LERR>
    error('Could not perform exploration setup');    
end