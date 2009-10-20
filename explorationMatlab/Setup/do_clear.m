% this is the do_clear function sets up most stuff
function do_clear(handles)

% MANUAL CHECKS: To do before start work
display('Make sure you have done the following');
display('1) Checked the laser is on by running the checklaser.exe file (power cycle if necessary)');
display('2) The ROBOT is powered on');
display('3) The MOTOR power is on');
display('4) Both EMERGENCY STOPs are not on');
display('5) Robot is in automatic mode on both pendant and E-stop control');
display('6) The robocontroller program is running on pendant and recieveing no communication');
display('7) There is no EyeinHand exe running intially in task manager');

display('Pausing so you can do these things');
pause


% Clear global variables
clear global workspace G_scan bestviews Q r densoobj all_views robot_maxreach classunkn_optimise alldirectedpoints graf_obs;

% try and delete coms
try COM_Delete_Coms(gcf);end %#ok<TRYNC>


% run the setup functions
if nargin==0;
    setupexplorationmain();
else
    setupexplorationmain(handles);
end

