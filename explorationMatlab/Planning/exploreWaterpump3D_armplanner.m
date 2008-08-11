function exploreWaterpump3D_armplanner()
close all;
global r Q workspace 

global sizeofthispath

% load('Journal/test9hMesh.mat')
% workspace.indexedobsticles=unique(workspace.inc_size*round(hMeshdata.v(GetimpLevInfo(hMeshdata.v),:)/workspace.inc_size),'rows');

%clear global graf_obs;
qlimits=r.qlim;


animate=false;sizeofthispath=1;
numdests=200;
numdust=1000;

matsize=[23,15,16];
%matsize=[24,16,1];
%matsize=[10,10,5];
table=zeros([matsize(1),matsize(2),matsize(3)]);
numofits=100;


startN=[round(matsize(1)/2),round(matsize(2)*3/4),round(matsize(3)/2)];
endN=ceil([rand(numdests-1,1)*matsize(1),rand(numdests-1,1)*matsize(2),rand(numdests-1,1)*matsize(3)]);

%make sure they are different and not zero
% while startN(1)==0 || startN(2)==0 || startN(3)==0 ; startN=round(rand(1,3)*matsize); end
% while (endN(1)==startN(1) && endN(2)==startN(2) && endN(3)==startN(3)) || endN(1)==0 || endN(2)==0 ||endN(3)==0 ;  endN=round(rand(1,3)*matsize); end

%% define workspace
indexed_knowncoords=round(setdiff(workspace.knowncoords(GetImpLevInfo(workspace.knowncoords),:),workspace.indexedobsticles,'rows')/workspace.inc_size);
%this makes the check for a collision quicker
obsticle_points=workspace.indexedobsticles(GetImpLevInfo(workspace.indexedobsticles),:);
all_possible=round(workspace.unknowncoords(workspace.lev1unknown   ,:)/workspace.inc_size);
[nothing,index]=setdiff(all_possible,[indexed_knowncoords;obsticle_points],'rows');
% unknown_points=workspace.unknowncoords(workspace.lev1unknown(index),:);
unknown_points=[];
display('Unknown points set to nothing for this exercise');

%% start the figures
figure(1)
subplot(1,2,2);camlight;axis equal;hold on;view(3)
plot3(obsticle_points(:,1),obsticle_points(:,2),obsticle_points(:,3),'marker','.','Color',[.2,.2,.1],'linestyle','none');
% plot3(unknown_points(:,1),unknown_points(:,2),unknown_points(:,3),'marker','.','Color',[0.9,0.9,0.9],'linestyle','none');
subplot(1,2,1);
xlabel('Joint 1 Increments');ylabel('Joint 2 Increments');zlabel('Joint 3 Increments')
axis([1 matsize(1) 1 matsize(2) 0 matsize(3)]);
view(3)


%% Make obstacles
tic
global graf_obs;


%if we dont have graf_obs object then build one
if isempty(graf_obs)
    graf_obs=[];
    for i=1:matsize(1)
        for j=1:matsize(2)
            for k=1:matsize(3)
                [J1,J2,J3]=mapindextojoints(i,j,k,qlimits,matsize);
                [results,unknown_points_result]=check_path_for_col([J1,J2,J3,0,0,0],obsticle_points,unknown_points);
                if ~(results && unknown_points_result)
                    graf_obs=[graf_obs;[i,j,k]];
                end
            end
        end
    end
else %else remove points that are now free from the old one 
    for i=1:matsize(1)
        for j=1:matsize(2)
            for k=1:matsize(3)
                if ~isempty(find(graf_obs(:,1)==i & graf_obs(:,2)==j & graf_obs(:,3)==k,1))
                    [J1,J2,J3]=mapindextojoints(i,j,k,qlimits,matsize);
                    [results,unknown_points_result]=check_path_for_col([J1,J2,J3,0,0,0],obsticle_points,unknown_points);
                    if results && unknown_points_result
                        nowfreeindx=find(graf_obs(:,1)==i & graf_obs(:,2)==j & graf_obs(:,3)==k,1);
                        graf_obs=graf_obs([1:nowfreeindx-1,nowfreeindx+1:end],:);
                    end
                end
            end
        end
    end    
end;


%spiral
% obdensity=8;
% t = 0:0.1:obdensity*matsize; graf_obs=unique(round([([t .* cos(t)]'+obdensity*matsize)./obdensity, ([t .* sin(t)]'+obdensity*matsize)./obdensity]),'rows');

%random stalagmite & stalactite like graf_obs
% for i=1:4:matsize
%     if rand>0.5
%         barlength=round(rand*matsize(1));while barlength==matsize(1); round(rand*matsize(1));end
%         current_z=1;
%         while rand>0.05
%             graf_obs=[graf_obs;[[1:matsize(1)-barlength]',i*ones([matsize(2)-barlength,1]),current_z*ones([matsize(3)-barlength,1])]];
%             current_z=current_z+1;
%         end
%     end
% end
% 
% for i=3:4:matsize
%     if rand>0.5
%         barlength=round(rand*matsize(1));while barlength==matsize(1); round(rand*matsize(1));end
%         current_z=0;
%         while rand>0.05
%             graf_obs=[graf_obs;[[barlength+1:matsize(1)+1]',i*ones([matsize(2)-barlength+1,1]),current_z*ones([matsize(3)-barlength+1,1])]];
%             current_z=current_z+1;
%         end       
%     end
% end

% random dust
% graf_obs=[graf_obs;ceil([rand(numdust-1,1)*matsize(1),rand(numdust-1,1)*matsize(2),rand(numdust-1,1)*matsize(3)])];

%remove obstacles that are on our start or end nodes
% if ~isempty(find(graf_obs(:,1)==startN(1) & graf_obs(:,2)==startN(2) & graf_obs(:,3)==startN(3) ,1))
%     badnode=find(graf_obs(:,1)==startN(1) & graf_obs(:,2)==startN(2) & graf_obs(:,3)==startN(3));
%     graf_obs=graf_obs([1:badnode-1,badnode+1:end],:);
% end
% if ~isempty(find(graf_obs(:,1)==endN(1) & graf_obs(:,2)==endN(2) & graf_obs(:,3)==endN(3),1))
%     badnode=find(graf_obs(:,1)==endN(1) & graf_obs(:,2)==endN(2) & graf_obs(:,3)==endN(3));
%     graf_obs=graf_obs([1:badnode-1,badnode+1:end],:);
% end

%get rid of zeros
graf_obs=graf_obs(graf_obs(:,1)>0 & graf_obs(:,2)>0 & graf_obs(:,3)>0 &...
                    graf_obs(:,1)<=matsize(1) & graf_obs(:,2)<=matsize(2) & graf_obs(:,3)<=matsize(3),:);
% plot(graf_obs(:,1),graf_obs(:,2),'r.');

%set start to be 1
table(startN(1),startN(2),startN(3))=1;
toc

%% Realise water from start
tic
for i=1:numofits; 
    a=[];b=[];c=[];
    totalsize=length(find(table==1));
    for cCount=1:matsize(3)
        if size(a,1)==totalsize;break;end
        [atemp,btemp]=find(table(:,:,cCount)==1);
        ctemp=cCount*ones([size(atemp,1),1]);
        a=[a;atemp];
        b=[b;btemp];
        c=[c;ctemp];
    end
    
    
%    if there are none ==1 then we have explored as much as possible
    if isempty(a);  
        display('Not all paths are possible');break; 
    end
    %set to update the surrounding cells
    toupdate=[a-1,b-1,c-1;a-1,b,c-1;a-1,b+1,c-1;
              a,b-1,c-1;a,b,c-1;a,b+1,c-1;
              a+1,b-1,c-1;a+1,b,c-1;a+1,b+1,c-1;
              
              a-1,b-1,c;a-1,b,c;a-1,b+1,c;
              a,b-1,c;a,b,c;a,b+1,c;
              a+1,b-1,c;a+1,b,c;a+1,b+1,c;
              
              a-1,b-1,c+1;a-1,b,c+1;a-1,b+1,c+1;
              a,b-1,c+1;a,b,c+1;a,b+1,c+1;
              a+1,b-1,c+1;a+1,b,c+1;a+1,b+1,c+1];
          
          toupdate=toupdate(toupdate(:,1)>=1 & toupdate(:,1)<=matsize(1),:);
          toupdate=toupdate(toupdate(:,2)>=1 & toupdate(:,2)<=matsize(2),:);
          toupdate=toupdate(toupdate(:,3)>=1 & toupdate(:,3)<=matsize(3),:);
          
    %find all other cells which are greater than 0 to make sure we don't
    %update them twice
    [a2,b2,c2]=find(table>1);
    
    %put together into a single index and only use unique
    singleindex=unique([(toupdate(:,3)-1)*matsize(1)*matsize(2)+(toupdate(:,2)-1)*matsize(1)+toupdate(:,1);...
                        (c2-1)*matsize(1)*matsize(2)^2+(b2-1)*matsize(1)+a2]);
    %only update each cell once         
    %toupdate=unique([toupdate;[c,d]],'rows');
    
    table(singleindex)=table(singleindex)+1;
    
    %remove graf_obs (set to 0)
    table((graf_obs(:,3)-1)*matsize(1)*matsize(2)+(graf_obs(:,2)-1)*matsize(1)+graf_obs(:,1))=0;
    
    %if we have got to the destination then break - a path is possible
    if isempty(find(table((endN(:,3)-1)*matsize(1)*matsize(2)+(endN(:,2)-1)*matsize(1)+endN(:,1))==0,1)); break; end
    %draw every now and again
    if  animate==true && rand>0.95 
        Xtoplot=1:matsize(1);
        Ytoplot=1:matsize(2);
        Ztoplot=1:matsize(3);
        toplot=[];plotcolor=[];
        for x=Xtoplot
            for y=Ytoplot
                toplot=[toplot;x*ones([matsize(3),1]),y*ones([matsize(3),1]),Ztoplot'];
                plotcolor=[plotcolor;squeeze(table(x,y,:))];
            end
        end
        plotcolor=plotcolor/max(plotcolor);        
        for j=0.1:0.1:1
            try delete(plothandle(j/0.1)); end
            covered=find(plotcolor+0.1>j & plotcolor<=j);
            if ~isempty(covered)
                hold on;
                plothandle(int32(j/0.1))=plot3(toplot(covered,1),toplot(covered,2),toplot(covered,3),'color',[1-j,1-j,1-j],'marker','.','linestyle','none');                
                drawnow;
            end
        end
%         saveas(gcf,['Robot3Dmovie_i',num2str(i),'_index',num2str(1000+sizeofthispath),'.png']);sizeofthispath=sizeofthispath+1;
    end

end
toc
% keyboard
%% Go from destination to source (ANIMATE)

%if there is a way to get to the start
if ~isempty(find(table((endN(:,3)-1)*matsize(1)*matsize(2)+(endN(:,2)-1)*matsize(1)+endN(:,1))>0,1))
    %plot start and finish
    hold on;
    plot3(startN(3),startN(2),startN(1),'r*')
    plot3(endN(:,3),endN(:,2),endN(:,1),'g*')
    plot3(graf_obs(:,3),graf_obs(:,2),graf_obs(:,1),'y.')
    scrsz = get(0,'ScreenSize');
    set(gcf,'Position',[1 scrsz(2) scrsz(3) scrsz(4)])
    axis equal;view(3);rotate3d

    goalsfound=find(table((endN(:,3)-1)*matsize(1)*matsize(2)+(endN(:,2)-1)*matsize(1)+endN(:,1))>0)';
    for i=1:size(goalsfound,2)
        %work backwards from the finish
        pathval(i).val=[endN(goalsfound(i),1),endN(goalsfound(i),2),endN(goalsfound(i),3)];
        maxval=0;
        % while the first step of the path is not at the start work backwards
        while pathval(i).val(1,1)~=startN(1) || pathval(i).val(1,2)~=startN(2) || pathval(i).val(1,3)~=startN(3)
            a=pathval(i).val(1,1);b=pathval(i).val(1,2);c=pathval(i).val(1,3);

            %find the next (actally previous) node can be equal to the same node
            nextnode=[a-1,b-1,c-1;a-1,b,c-1;a-1,b+1,c-1;
                      a,b-1,c-1;a,b,c-1;a,b+1,c-1;
                      a+1,b-1,c-1;a+1,b,c-1;a+1,b+1,c-1;

                      a-1,b-1,c;a-1,b,c;a-1,b+1,c;
                      a,b-1,c;a,b+1,c;
                      a+1,b-1,c;a+1,b,c;a+1,b+1,c;

                      a-1,b-1,c+1;a-1,b,c+1;a-1,b+1,c+1;
                      a,b-1,c+1;a,b,c+1;a,b+1,c+1;
                      a+1,b-1,c+1;a+1,b,c+1;a+1,b+1,c+1];
              
            nextnode=nextnode(nextnode(:,1)>=1 & nextnode(:,1)<=matsize(1),:);
            nextnode=nextnode(nextnode(:,2)>=1 & nextnode(:,2)<=matsize(2),:); 
            nextnode=nextnode(nextnode(:,3)>=1 & nextnode(:,3)<=matsize(3),:); 
            %not exqual to the 
            if size(pathval(i).val,1)>1
%                 nextnode=nextnode(~(nextnode(:,1)==pathval(i).val(2,1)&nextnode(:,2)==pathval(i).val(2,2)& nextnode(:,3)==pathval(i).val(2,3)),:);
                nextnode=setdiff(nextnode,pathval(i).val,'rows');
            end
            %go through possible nextnodes and find the highest or the closest if they are the same value
            index=1;updated=false;
            for j=1:size(nextnode,1)
                if table(nextnode(j,1),nextnode(j,2),nextnode(j,3))>maxval 
                    index=j;maxval=table(nextnode(j,1),nextnode(j,2),nextnode(j,3));
                    updated=true;
                elseif table(nextnode(j,1),nextnode(j,2),nextnode(j,3))==maxval &&...
                        ((startN(1)-nextnode(j,1))^2+(startN(2)-nextnode(j,2))^2+(startN(3)-nextnode(j,3))^2)<=...
                        ((startN(1)-nextnode(index,1))^2+(startN(2)-nextnode(index,2))^2)+(startN(3)-nextnode(index,3))^2
                    index=j;maxval=table(nextnode(j,1),nextnode(j,2),nextnode(j,3));
                    updated=true;
                end
            end

            pathval(i).val=[nextnode(index,:);pathval(i).val];

            %plot the pathval        
            if animate
                plot3(nextnode(index,3),nextnode(index,2),nextnode(index,1),'r.');
                drawnow;pause(0.05);                
            end
        end
        
        
        %check the inbetween nodes of the path
        pathval(1).allsteps=[];
        for pnode=1:size(pathval(i).val,1)-1
            if pnode==1;
                [J1,J2,J3]=mapindextojoints(pathval(i).val(1,1),pathval(i).val(1,2),pathval(i).val(1,3),qlimits,matsize);
                Q=[J1,J2,J3,zeros(size(J3,1),1),zeros(size(J3,1),1),zeros(size(J3,1),1)];
            else                
                [J1,J2,J3]=mapindextojoints(pathval(i).val(pnode-1,1),pathval(i).val(pnode-1,2),pathval(i).val(pnode-1,3),qlimits,matsize);
                Q=[J1,J2,J3,zeros(size(J3,1),1),zeros(size(J3,1),1),zeros(size(J3,1),1)];
            end
            pathval(i).result=1;
            [J1,J2,J3]=mapindextojoints(pathval(i).val(pnode+1,1),pathval(i).val(pnode+1,2),pathval(i).val(pnode+1,3),qlimits,matsize);
            newQ=[J1,J2,J3,zeros(size(J3,1),1),zeros(size(J3,1),1),zeros(size(J3,1),1)];
            [result,tempsteps]=pathplanner_new(newQ,0,1,0,0,0);
            if result==1
                pathval(i).allsteps=[pathval(i).allsteps;tempsteps];
            else
                pathval(i).result=0;
            end
        end
        
        
        
        %show the path lines
        plot3(pathval(i).val(:,3),pathval(i).val(:,2),pathval(i).val(:,1),'r');
        plot3(pathval(i).val(:,3),pathval(i).val(:,2),pathval(i).val(:,1),'r');
        
        %Plot the robot path
        [J1,J2,J3]=mapindextojoints(pathval(i).val(:,1),pathval(i).val(:,2),pathval(i).val(:,3),qlimits,matsize);
        subplot(1,2,2);
        demopath_new([J1,J2,J3,zeros(size(J3,1),1),zeros(size(J3,1),1),zeros(size(J3,1),1)])
        subplot(1,2,1);
    end   
    
%     %show pathval at end
%     if ~animate
%         hold on
%         for i=1:1:size(pathval,2)
%             plot3(pathval(i).val(:,3),pathval(i).val(:,2),pathval(i).val(:,1),'r.');            
%         end
%     end
else
    display('Make that none');
end







%this maps from index to joint limits
function [J1,J2,J3]=mapindextojoints(a,b,c,qlimits,matsize)
J1=[];J2=[];J3=[];
for i=1:size(a,1)
    temp=[a(i),b(i),c(i)]'.*(qlimits(1:3,2)-qlimits(1:3,1))./matsize'+qlimits(1:3,1);
    J1=[J1;temp(1)];J2=[J2;temp(2)];J3=[J3;temp(3)];
end