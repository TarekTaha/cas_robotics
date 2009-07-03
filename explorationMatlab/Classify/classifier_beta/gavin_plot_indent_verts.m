%% gavin_spacial_correctness
%
% *Description:* Plot the classified data back onto the vertices 
%
% *Inputs*
% subset_all_data (3xmany) with angle of incidence, ormalised intensity and rang, 
% subset_all_data_T (1xmany) the correct answer as we specified
% subset_all_data_xyz (3xmany) xyz points which correspond to the data
% winner (int 0->nummats) 0 is unzure then 1-> num mats for that material

function gavin_plot_indent_verts(subset_all_data,subset_all_data_T,subset_all_data_xyz,winner)
close all
colors=['k','r','g','b'];
for currmat=0:3
    index=find(winner==currmat);
    if size(index,1)>0
        plot3(subset_all_data_xyz(index,1),subset_all_data_xyz(index,2),subset_all_data_xyz(index,3),[colors(currmat+1),'.']);
    end
    hold on;
end

axis equal
view(3)
grid on;