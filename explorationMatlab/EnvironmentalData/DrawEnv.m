%% DrawEnv
% Desciption: This function plots the cart and rails environment I change,
% the environment was originally built by tom

function DrawEnv(add1_remove0)

if nargin<1
    add1_remove0=1;
end

guiParams=getappdata(gcf,'guiParams');

if add1_remove0==1
    deletehandles()

    load ('ObjectsPlatform2.mat');
    guiParams.cartnRails=inf;
    
    hold on;
   
    guiParams.cartnRails(end+1)=patch('Vertices',Objects.E1,'Faces',Objects.FE1,'FaceColor',[1,1,0.5],'EdgeColor',[0,0.5,0.5]);
    set(guiParams.cartnRails(end),'FaceColor',[1,1,0.8],'EdgeColor','none');

    guiParams.cartnRails(end+1)=patch('Vertices',Objects.O1,'Faces',Objects.FO1,'FaceColor',[0,0,0.5],'EdgeColor',[0,0.5,0.5]);
    set(guiParams.cartnRails(end),'FaceColor',[0,0,0.5],'EdgeColor','none');
else
    deletehandles()
end
setappdata(gcf,'guiParams',guiParams);                                 

%% try and delete all the handles
function deletehandles()
guiParams=getappdata(gcf,'guiParams');
if isfield(guiParams,'cartnRails')
    for i=1:size(guiParams.cartnRails,2)
        try delete(guiParams.cartnRails(i));
        end
    end
end
guiParams.cartnRails=[];
setappdata(gcf,'guiParams',guiParams);  



