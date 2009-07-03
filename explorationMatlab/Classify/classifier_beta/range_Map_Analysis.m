%% range_Map_Analysis
% *Description:* 
% Use steves boundary data from the mesh map to trace around the edges of
% the messh trianges

function range_Map_Analysis(hMesh)


    v = hMesh.VertexData;
    b = hMesh.BoundaryData;
    r = unique(b(:,2)); % Extract region identifiers
    figure;hold on
    cm = colormap;
    for k=1:length(r)
        c = ceil(rand*size(cm,1)); % Select a color
        vi = b(b(:,2)==r(k),1); % Extract the vertex indices
        plot3(v(vi,1), v(vi,2), v(vi,3), 'Color', cm(c,:));
    end

