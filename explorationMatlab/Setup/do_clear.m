% this is the do_clear function clears globals and sets up most stuff
function do_clear()

% Clear global variables
clear global workspace G_scan bestviews Q r densoobj all_views robot_maxreach optimise classunkn_optimise classifiedplotHa alldirectedpoints graf_obs;

% try and delete coms
try 
    COM_Delete_Coms(gcf);
catch %#ok<CTCH>
    display('Some problems running COM_Delete_Coms in do_clear');
end 


% Deleting all timers
display('Deleting all timers');
delete(timerfind());
