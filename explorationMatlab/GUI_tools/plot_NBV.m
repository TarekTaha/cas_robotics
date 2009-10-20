function plot_NBV(selectionORhandles)

global bestviews;

if isstruct('selectionORhandles') %assuming exGUI was passed
    selection=get(handles.best_NBV_listbox,'value');
else %assuming that a bestview selection was passed
    selection=selectionORhandles;
end

if bestviews(selection).valid

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
else
    msgbox('Not a valid robot pose selected')
end