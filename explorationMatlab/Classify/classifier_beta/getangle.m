%% getangle
% *Description:* 
% 1) Works out the surfaces due to the verticy data with PCA
% 2) Angle on incidence of each point with the surface
function [vertices,source,intestity,angleincidence,scanrange]=getangle(mu,vertices,normals,scanrange,source,intestity,positiondata)

global config

tostopat=150;
do_plot=false;
% Min and max ray as requested  
minray=config.minray;
maxray=config.maxray;

angleincidence=inf*ones([size(vertices,1),1]);
    
%make surfaces and collect all homepoints and normals
tic;surface_making_simple(vertices,mu);toc
global plane
plane_homepoints=zeros([size(plane,2),3]);
plane_normals=zeros([size(plane,2),3]);
plane_for_vertices=zeros([size(vertices,1),1]);

%extract info from planes make sure normals are correct way
for i=1:size(plane,2)
    plane_homepoints(i,:)=plane(i).home_point;
    
    if i==tostopat && do_plot
        figure(1)
        plot3(plane_homepoints(i,1),plane_homepoints(i,2),plane_homepoints(i,3),'k*');
        hold on;
    end
    
    %get a vector from plane a one from averaging 
    vec1=plane(i).normal_by_eigenval';
    vec1=vec1/norm(vec1);      
    
    if i==tostopat && do_plot
        plot3(plane_homepoints(i,1)+vec1(1),...
              plane_homepoints(i,2)+vec1(2),...
              plane_homepoints(i,3)+vec1(3),'r*');
        hold on;
        plot3([plane_homepoints(i,1),plane_homepoints(i,1)+vec1(1)],...
              [plane_homepoints(i,2),plane_homepoints(i,2)+vec1(2)],...
              [plane_homepoints(i,3),plane_homepoints(i,3)+vec1(3)],'r');
    end

    %average across the normals to see which direction it is in
%     vec2=plane(i).home_point+sum(normals(plane(i).points,:),1)/size(plane(i).points,1);
    vec2=sum(normals(plane(i).points,:),1)/size(plane(i).points,1);
    %normalise
    vec2=vec2/norm(vec2);
    
    if i==tostopat && do_plot
        plot3(plane_homepoints(i,1)+vec2(1),...
              plane_homepoints(i,2)+vec2(2),...
              plane_homepoints(i,3)+vec2(3),'b*');
        plot3([plane_homepoints(i,1),plane_homepoints(i,1)+vec2(1)],...
              [plane_homepoints(i,2),plane_homepoints(i,2)+vec2(2)],...
              [plane_homepoints(i,3),plane_homepoints(i,3)+vec2(3)],'b');
          for temp_count=1:size(plane(i).points,1)
              plot3([vertices(plane(i).points(temp_count),1),vertices(plane(i).points(temp_count),1)+normals(plane(i).points(temp_count),1)],...
                    [vertices(plane(i).points(temp_count),2),vertices(plane(i).points(temp_count),2)+normals(plane(i).points(temp_count),2)],...
                    [vertices(plane(i).points(temp_count),3),vertices(plane(i).points(temp_count),3)+normals(plane(i).points(temp_count),3)],'g');
          end
          axis equal
    end
    
    % they should be in the same direction
    if acos(dot(vec1,vec2))<pi/2
        plane_normals(i,:)=plane(i).normal_by_eigenval';
        
        if i==tostopat
            title('selecting same direction')
        end
        
    else %need to reverse the direction of the normal
        plane_normals(i,:)=-plane(i).normal_by_eigenval';
        
        if i==tostopat
            title('selecting different direction')
        end

    end
    
    %register each vertciy to a plane which holds is 
    plane_for_vertices(plane(i).points)=i;
end    

%make sure all vertices have plane if not put in a closest plane
for i=find(plane_for_vertices==0)'
    [nothing, index]=min((vertices(i,1)-plane_homepoints(:,1)).^2 + ...
                         (vertices(i,2)-plane_homepoints(:,2)).^2 + ...
                         (vertices(i,3)-plane_homepoints(:,3)).^2);
    plane_for_vertices(i)=index;
end

%determine angle of incidence to plane for each point
for i=1:max(source(:,6))
    scanorigin(i).positiondata=[positiondata.motion.origin_x_first(i)+positiondata.motion.origin_x_last(i),...
                                positiondata.motion.origin_y_first(i)+positiondata.motion.origin_y_last(i),...
                                positiondata.motion.origin_z_first(i)+positiondata.motion.origin_z_last(i)]/2;   
    
    if i==tostopat && do_plot            
        display([num2str(positiondata.motion.origin_x_first(i)),',',num2str(positiondata.motion.origin_y_first(i)),',',num2str(positiondata.motion.origin_z_first(i)),'...',num2str(positiondata.motion.origin_x_last(i)),',',num2str(positiondata.motion.origin_y_last(i)),',',num2str(positiondata.motion.origin_z_last(i))]);
        display(['Result pos=',num2str(scanorigin(i).positiondata)]);
        figure(2)
        plot3(scanorigin(i).positiondata(1),scanorigin(i).positiondata(2),scanorigin(i).positiondata(3),'g*');
    end
    touse=(source(:,6)==i & source(:,5)>minray & source(:,5)<maxray);   
    %         tempvector=[vertices(touse,1)-scanorigin(i).val(1),vertices(touse,2)-scanorigin(i).val(2),vertices(touse,3)-scanorigin(i).val(3)];
    tempvector=[scanorigin(i).positiondata(1)-vertices(touse,1),...
                scanorigin(i).positiondata(2)-vertices(touse,2),...
                scanorigin(i).positiondata(3)-vertices(touse,3)];            
            
    if i==tostopat && do_plot
        hold on;
        count=1;
        plot3(vertices(touse,1),vertices(touse,2),vertices(touse,3),'color','b','marker','.','markersize',4,'linestyle','none');
              
%         for temp_count=find(touse)'
%             plot3([vertices(temp_count,1),vertices(temp_count,1)+tempvector(count,1)],...
%                   [vertices(temp_count,2),vertices(temp_count,2)+tempvector(count,2)],...
%                   [vertices(temp_count,3),vertices(temp_count,3)+tempvector(count,3)],'k');
%               count=count+1;
%         end
        keyboard
    end
    
    norm_length=sqrt(sum(tempvector.^2,2));
    tempvector=[tempvector(:,1)./norm_length,...
                tempvector(:,2)./norm_length,...
                tempvector(:,3)./norm_length];
            
    angleincidence(touse)=acos(dot(tempvector,plane_normals(plane_for_vertices(touse),:),2))*180/pi;    
end

%discard (i.e. set back to original inf (ignore) if angle is greater than 90' since it is obviously a stupid plane
angleincidence(find(angleincidence>90 & angleincidence<inf))=Inf;

 %resize all to remove crap from outskirts    
vertices=vertices(angleincidence<inf,:);
source=source(angleincidence<inf,:);
intestity=intestity(angleincidence<inf,:);
angleincidence=angleincidence(angleincidence<inf,:);
scanrange=scanrange(angleincidence<inf);
end