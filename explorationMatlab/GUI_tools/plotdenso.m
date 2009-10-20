%% plotdenso
%
% *Description:*  pass in the robot 'r' and the joints 'Q' and the tris specs
% for each piece Denso02  held in densoobj global var also hold plot data

%% Function Call
% 
% *Inputs:* 
%
% _r_ (object) robot object - want to pass not use global??
%
% _Q_ (1*6 double) radians, current joint configuration 
%
% _checkFF_(binary) whether or not to recheck the forcefields as we plot
%
% _plot_ellipse_(binary) wether or not to show the ellipses as well as robot
%
% *Returns:* NULL

function plotdenso(r, Q, checkFF, plot_ellipse)

%% Variables
global densoobj robot_maxreach

if ~isfield(densoobj,'M') || ~isfield(densoobj,'F')
    error('Either densoobj does not exist as a global variable or there is no densoobj.mat file in the correct format')
end

%set defaults for if we want to check the FF and plot elispes
if nargin<4
    plot_ellipse=0; %display('plot_ellipse is set to false by default');
    if nargin<3
        checkFF=0; %display('checkFF is set to false by default');
    end
end
    
%this will hold the pieves in collision only if we are plotting the ellipse
pieces_in_collision=[];

%try and delete the old patches and plots
try for piece=1:size(densoobj,2); delete(densoobj(piece).patches); end; end
try for piece=1:size(densoobj,2); delete(densoobj(piece).ellipse.plothandle); end; end


%if you want to show the joints instead/aswell
%plot(r,Q,'joints')

%find out where robot is now
n = r.n;
L = r.link;
t = r.base;
ellipse_safetyfactor=robot_maxreach.ellipse_safetyfactor;

%% do stephens way
try;plot(r,Q,'axis',gcf);
catch;plot(r,Q);
end
if checkFF % if we have to check the FF
    for piece=1:r.n
        t = t * L{piece}(Q(piece));
        %If ellipses don't exist
        if ~isfield(densoobj,'ellipse')
            [densoobj(piece).ellipse.x,densoobj(piece).ellipse.y,densoobj(piece).ellipse.z,...
                        densoobj(piece).ellipse.params,densoobj(piece).ellipse.center]=calc_elip(piece,L{piece}.glyph.vertices,ellipse_safetyfactor);
        end
        
        temp_ellipse_vals=densoobj(piece).ellipse;
        
        %Put all mesh data for x, y, z in three cols (so it can be translated)
        allx=[];ally=[];allz=[];
        for i=1:size(temp_ellipse_vals.x,2);allx=[allx;temp_ellipse_vals.x(:,i)];end
        for i=1:size(temp_ellipse_vals.y,2);ally=[ally;temp_ellipse_vals.y(:,i)];end
        for i=1:size(temp_ellipse_vals.z,2);allz=[allz;temp_ellipse_vals.z(:,i)];end        
        
        %translate the ellipse data
        ellipse_data=(t(1:3,1:3)*[allx,ally,allz]')';
        ellipse_data=[ellipse_data(:,1)+t(1,4) ellipse_data(:,2)+t(2,4) ellipse_data(:,3)+t(3,4)];

        %break up (for x, y, z) - for the mesh
        allx=ellipse_data(:,1); ally=ellipse_data(:,2); allz=ellipse_data(:,3);
        breakupsize=[size(temp_ellipse_vals.x,1),size(temp_ellipse_vals.y,1),size(temp_ellipse_vals.z,1)];
        %put back in mesh
        for i=1:size(temp_ellipse_vals.x,2);temp_ellipse_vals.x(:,i)=allx((i-1)*breakupsize(1)+1:i*breakupsize(1));end;
        for i=1:size(temp_ellipse_vals.y,2);temp_ellipse_vals.y(:,i)=ally((i-1)*breakupsize(2)+1:i*breakupsize(2));end;
        for i=1:size(temp_ellipse_vals.z,2);temp_ellipse_vals.z(:,i)=allz((i-1)*breakupsize(3)+1:i*breakupsize(3));end;
        
        if plot_ellipse
            %so the ellipses are see through plot them with the new transformed data
            hold on;
            densoobj(piece).ellipse.plothandle=mesh(temp_ellipse_vals.x,temp_ellipse_vals.y,temp_ellipse_vals.z);
        end      
%         if(piece<n); t = t * L{piece}(Q(piece));end
    end    
    if piece>1; %check all but the first peice
        result=check_FF(t,densoobj(piece).ellipse);
        if result==0; pieces_in_collision=[pieces_in_collision,piece];end;
    end
end
   
            

% %% For each piece (base,1->6) plot patch, transform (rot, place) robot pose
% for piece=1:7
%     verts = (t(1:3,1:3)*densoobj(piece).M')';    
%     verts = [verts(:,1)+t(1,4) verts(:,2)+t(2,4) verts(:,3)+t(3,4)];
% % if piece==2
% %     verts(:,3)= verts(:,3)+0.015;
% % end
%     
%     %plot the pieces
%     if piece==1;     densoobj(piece).patches=patch('Vertices',verts,'Faces',densoobj(piece).F,'FaceColor',[0, 0, 0.7],'EdgeColor',[0 0 0.8]);
%         set(densoobj(piece).patches,'FaceColor',[0.4,0.4,1],'EdgeColor','None');
%     elseif piece==7; densoobj(piece).patches=patch('Vertices',verts,'Faces',densoobj(piece).F,'FaceColor',[0.7, 0, 0],'EdgeColor',[0.7, 0, 0]);
%         set(densoobj(piece).patches,'FaceColor',[0.7, 0, 0],'EdgeColor','None');
%     else             densoobj(piece).patches=patch('Vertices',verts,'Faces',densoobj(piece).F,'FaceColor',[0.8, 0.8, 0.8],'EdgeColor',[0.4 0.4 0.4]);
%         set(densoobj(piece).patches,'FaceColor',[1, 1, 0.9],'EdgeColor','None');
%     end        
%     
%     
%     %if we want to see an ellipse
%     if checkFF
%         %if an elispe has not been made for this piece
%         if ~isfield(densoobj,'ellipse')
%             [densoobj(piece).ellipse.x,densoobj(piece).ellipse.y,densoobj(piece).ellipse.z,...
%                 densoobj(piece).ellipse.params,densoobj(piece).ellipse.center]=calc_elip(piece,densoobj(piece).M);            
%         end
% 
%         %so we don't alter the global use a temp here for transforming
%         temp_ellipse_vals=densoobj(piece).ellipse;
%         
%         %Put all mesh data for x, y, z in three cols (so it can be translated)
%         allx=[];ally=[];allz=[];
%         for i=1:size(temp_ellipse_vals.x,2);allx=[allx;temp_ellipse_vals.x(:,i)];end
%         for i=1:size(temp_ellipse_vals.y,2);ally=[ally;temp_ellipse_vals.y(:,i)];end
%         for i=1:size(temp_ellipse_vals.z,2);allz=[allz;temp_ellipse_vals.z(:,i)];end        
%         
%         %translate the ellipse data
%         ellipse_data=(t(1:3,1:3)*[allx,ally,allz]')';
%         ellipse_data=[ellipse_data(:,1)+t(1,4) ellipse_data(:,2)+t(2,4) ellipse_data(:,3)+t(3,4)];
%         
%         %break up (for x, y, z) - for the mesh
%         allx=ellipse_data(:,1); ally=ellipse_data(:,2); allz=ellipse_data(:,3);
%         breakupsize=[size(temp_ellipse_vals.x,1),size(temp_ellipse_vals.y,1),size(temp_ellipse_vals.z,1)];
%         %put back in mesh
%         for i=1:size(temp_ellipse_vals.x,2);temp_ellipse_vals.x(:,i)=allx((i-1)*breakupsize(1)+1:i*breakupsize(1));end;
%         for i=1:size(temp_ellipse_vals.y,2);temp_ellipse_vals.y(:,i)=ally((i-1)*breakupsize(2)+1:i*breakupsize(2));end;
%         for i=1:size(temp_ellipse_vals.z,2);temp_ellipse_vals.z(:,i)=allz((i-1)*breakupsize(3)+1:i*breakupsize(3));end;
%         
%         if plot_ellipse
%             %so the ellipses are see through plot them with the new transformed data
%             hold on;
%             densoobj(piece).ellipse.plothandle=mesh(temp_ellipse_vals.x,temp_ellipse_vals.y,temp_ellipse_vals.z);
%         end      
%         
%         %check the ellipse force fields to make sure there are no point
%         %inside Must pass in the original ellipse (pretranslated) so points
%         %can be reverse translated
%         if piece>1; 
%             result=check_FF(t,densoobj(piece).ellipse);
%             if result==0; pieces_in_collision=[pieces_in_collision,piece];end;
%         end
%     end
% 
%     %do next translation only up to 6 links since 6 DOF
%     if(piece<n) t = t * L{piece}(Q(piece));end
%         
% end

%% Draw hidden if we are showing ellipses to make them seethrough
if plot_ellipse
    hidden;
end

%% Simple display for if a collision exists
if checkFF && ~isempty(pieces_in_collision)
    display(strcat('The force field returned a collision on piece:',num2str(pieces_in_collision))); 
end