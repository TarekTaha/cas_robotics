%% MAIN setup function
% Only pass in handles variable if using exGUI
function setupexplorationmain(handles)

if nargin==0
    DOsetupGUI=false;    
    showunknownpnts=false;
else
    DOsetupGUI=true;
    try showunknownpnts=get(handles.show_unknownpoints_checkbox,'Value'); catch; showunknownpnts=false; end  %#ok<CTCH>
end

%% MANUAL CHECKS: To do before start work
display('Make sure:');
display('1) The ROBOT is powered on');
display('2) The MOTOR power is on');
display('3) Both EMERGENCY STOPs are not on');
display('4) Robot is in automatic mode on both pendant and E-stop control');
display('5) The robocontroller program is running on pendant and recieveing no communication');
display('6) If no EyeinHand exe expected then it is not running');

% display('Pausing so you can do these things');pause

%% Go through the steps of setup
try
    %set the paths used in exGUI
    setuppath()
    
    %setup the GUI
    if DOsetupGUI; setupGUI(handles); end

    %Sets up the robot
    setuprobot();

    % Sets up the scanner
    setupscanner();

    %this sets up the joint optimisation and NBV optimisation 
    setupoptimisation();
    
    %sets up the occupancy grid and other robot workspace related variables
    setupworkspace(showunknownpnts);

    % Create COMS (must be after the workspace is setup)
    setupCOMs(gcf);

catch  %#ok<CTCH>
    lasterr %#ok<LERR>
    error('Could not perform exploration setup');    
end