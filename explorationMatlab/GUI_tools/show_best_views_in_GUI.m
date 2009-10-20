%% show_best_views_in_GUI
% Desciprtion: This goes through bestvbiews and shows them in the exGUI.
% May be exstended to work with other GUIS

function show_best_views_in_GUI(handles)
global bestviews;

% Go through best views list and packages as text
for cur_view=1:size(bestviews,2)
    if bestviews(cur_view).valid
        bestview_stringcell{cur_view}=...
                ['#',num2str(cur_view),': info=',num2str(bestviews(cur_view).addinfo),', @(',num2str(bestviews(cur_view).tr(1:3,4)'),'),mw=',num2str(sum(sum(abs(diff(bestviews(cur_view).all_steps)))))];
     end
end

%display in the best_NBV_listbox listbox
try 
    set(handles.best_NBV_listbox,'string',bestview_stringcell,'Value',1);
    set(handles.best_NBV_listbox,'visible','on');
    set(handles.descripbestview_text,'visible','on');
catch %#ok<CTCH>
    lasterr
    error('Problem updating the GUI');
end
    