%% DrawEnv
% Desciption: This function plots the cart and rails environment I change,
% the environment was originally built by tom

function DrawEnv(add1_remove0)

if nargin<1
    add1_remove0=1;
end

global guiglobal

if add1_remove0==1
    deletehandles()

    load ('ObjectsPlatform2.mat');
    guiglobal.cartnRails=inf;
    
    hold on;
   
    guiglobal.cartnRails(end+1)=patch('Vertices',Objects.E1,'Faces',Objects.FE1,'FaceColor',[1,1,0.5],'EdgeColor',[0,0.5,0.5]);
    set(guiglobal.cartnRails(end),'FaceColor',[1,1,0.8],'EdgeColor','none');

    guiglobal.cartnRails(end+1)=patch('Vertices',Objects.O1,'Faces',Objects.FO1,'FaceColor',[0,0,0.5],'EdgeColor',[0,0.5,0.5]);
    set(guiglobal.cartnRails(end),'FaceColor',[0,0,0.5],'EdgeColor','none');
else
    deletehandles()
end
    

%% try and delete all the handles
function deletehandles()
global guiglobal
if isfield(guiglobal,'cartnRails')
    for i=1:size(guiglobal.cartnRails,2)
        try delete(guiglobal.cartnRails(i));
        end
    end
end

guiglobal.cartnRails=[];



