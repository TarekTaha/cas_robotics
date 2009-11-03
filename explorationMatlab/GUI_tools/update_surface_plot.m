%% update_surface_plot
% Description: deletes old surface handles and writes a new one saves back
% into the guiParams in the figure. 
% NOTE: only pass in handes if using exGUI otherwise dont

function update_surface_plot(handles)
global workspace

% Set the default workspace size
aabb = [workspace.min; workspace.max];

% Gets the gui paramters from the GUI
guiParams=getappdata(gcf,'guiParams');

% Get the hCOM object from the GUI
hCOM=getappdata(gcf,'hCOM');

% try and delete the previous mesh_h. If nonexistant a new one is created
try delete(guiParams.mesh_h);
catch %#ok<CTCH>
    
    %if handles was passed in then check if we want to show the whole mesh
    if nargin>0
    	try 
            set(gcf,'CurrentAxes',handles.axes3);
            % if we want to show the whole environment    
            if get(handles.all_mesh_checkbox,'value')==1
                aabb = [-20, -20, -20; 20, 20, 20];
            end
        end
    end
    
    % extract the mesh from the surface object
    hMesh = hCOM.Surface.SurfacesInsideBox(aabb(1,:), aabb(2,:));
    f = hMesh.FaceData;
    v = hMesh.VertexData;
    hold on;
    guiParams.mesh_h=trisurf(f, v(:,1), v(:,2), v(:,3), 'FaceColor', 'None');
    axis equal
end
% Update the guiParams in the GUI
setappdata(gcf,'guiParams',guiParams)