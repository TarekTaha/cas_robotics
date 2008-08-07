function exploreWaterpump3D()
close all;

takemovie=true; F=[];
animate=false;
numdests=20;

matsize=20; %50 is good
table=zeros([matsize,matsize,matsize]);
numofits=10000;

% startN=round(rand(1,2)*matsize);
startN=[1,matsize/2,1];
endN=[ceil(rand(numdests-1,3)*matsize)];

%make sure they are different and not zero
% while startN(1)==0 || startN(2)==0 || startN(3)==0 ; startN=round(rand(1,3)*matsize); end
% while (endN(1)==startN(1) && endN(2)==startN(2) && endN(3)==startN(3)) || endN(1)==0 || endN(2)==0 ||endN(3)==0 ;  endN=round(rand(1,3)*matsize); end

%% Make obstacles
obstacles=[];

%spiral
% obdensity=8;
% t = 0:0.1:obdensity*matsize; obstacles=unique(round([([t .* cos(t)]'+obdensity*matsize)./obdensity, ([t .* sin(t)]'+obdensity*matsize)./obdensity]),'rows');

%random stalagmite & stalactite like obstacles
for i=1:4:matsize
    if rand>0.5
        barlength=round(rand*matsize);while barlength==matsize; round(rand*matsize);end
        current_z=1;
        while rand>0.05
            obstacles=[obstacles;[[1:matsize-barlength]',i*ones([matsize-barlength,1]),current_z*ones([matsize-barlength,1])]];
            current_z=current_z+1;
        end
    end
end

for i=3:4:matsize
    if rand>0.5
        barlength=round(rand*matsize);while barlength==matsize; round(rand*matsize);end
        current_z=0;
        while rand>0.05
            obstacles=[obstacles;[[barlength+1:matsize+1]',i*ones([matsize-barlength+1,1]),current_z*ones([matsize-barlength+1,1])]];
            current_z=current_z+1;
        end       
    end
end

% random dust
% obstacles=[obstacles;round(rand(matsize^2,3)*matsize)];

if ~isempty(find(obstacles(:,1)==startN(1) & obstacles(:,2)==startN(2) & obstacles(:,3)==startN(3) ,1))
    badnode=find(obstacles(:,1)==startN(1) & obstacles(:,2)==startN(2) & obstacles(:,3)==startN(3));
    obstacles=obstacles([1:badnode-1,badnode+1:end],:);
end
if ~isempty(find(obstacles(:,1)==endN(1) & obstacles(:,2)==endN(2) & obstacles(:,3)==endN(3),1))
    badnode=find(obstacles(:,1)==endN(1) & obstacles(:,2)==endN(2) & obstacles(:,3)==endN(3));
    obstacles=obstacles([1:badnode-1,badnode+1:end],:);
end
%get rid of zeros
obstacles=obstacles(obstacles(:,1)>0 & obstacles(:,2)>0 & obstacles(:,3)>0 &...
                    obstacles(:,1)<=matsize & obstacles(:,2)<=matsize & obstacles(:,3)<=matsize,:);
% plot(obstacles(:,1),obstacles(:,2),'r.');

%set start to be 1
table(startN(1),startN(2),startN(3))=1;

%% Realise water from start
tic
for i=1:numofits;
    a=[];b=[];c=[];
    totalsize=length(find(table==1));
    for cCount=1:matsize
        if size(a,1)==totalsize;break;end
        [atemp,btemp]=find(table(:,:,cCount)==1);
        ctemp=cCount*ones([size(atemp,1),1]);
        a=[a;atemp];
        b=[b;btemp];
        c=[c;ctemp];
    end
    
    
%    if there are none ==1 then we have explored as much as possible
    if isempty(a);  
        display('No path possible');break; 
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
          
          toupdate=toupdate(toupdate(:,1)>0 & toupdate(:,1)<matsize+1,:);
          toupdate=toupdate(toupdate(:,2)>0 & toupdate(:,2)<matsize+1,:);
          toupdate=toupdate(toupdate(:,3)>0 & toupdate(:,3)<matsize+1,:);
          
    %find all other cells which are greater than 0
%     [c,d]=find(table>1);
    [a2,b2,c2]=find(table>1);
    
    %put together into a single index and only use unique
    singleindex=unique([(toupdate(:,3)-1)*matsize^2+(toupdate(:,2)-1)*matsize+toupdate(:,1);...
                        (c2-1)*matsize^2+(b2-1)*matsize+a2]);
    %only update each cell once         
    %toupdate=unique([toupdate;[c,d]],'rows');
    
    table(singleindex)=table(singleindex)+1;
    
    %remove obstacles (set to 0)
    table((obstacles(:,3)-1)*matsize^2+(obstacles(:,2)-1)*matsize+obstacles(:,1))=0;
    
    %if we have got to the destination then break - a path is possible
    if isempty(find(table((endN(:,3)-1)*matsize^2+(endN(:,2)-1)*matsize+endN(:,1))==0,1)); break; end
    %draw every now and again
    if  animate==true && rand>1 %0.95 
        Xtoplot=1:matsize;
        Ytoplot=1:matsize;
        Ztoplot=1:matsize;
        toplot=[];plotcolor=[];
        for x=Xtoplot
            for y=Ytoplot
                toplot=[toplot;x*ones([matsize,1]),y*ones([matsize,1]),Ztoplot'];
                plotcolor=[plotcolor;squeeze(table(x,y,:))];
            end
        end
        plotcolor=plotcolor./max(plotcolor);
        scatter3(toplot(:,1),toplot(:,2),toplot(:,3),plotcolor','marker','*');
        drawnow
    end

end
toc
% keyboard
%% Go from destination to source (ANIMATE)

%if there is a way to get to the start
if ~isempty(find(table((endN(:,3)-1)*matsize^2+(endN(:,2)-1)*matsize+endN(:,1))>0,1))
    %plot start and finish
    hold on;
    plot3(startN(3),startN(2),startN(1),'r*')
    plot3(endN(:,3),endN(:,2),endN(:,1),'g*')
    plot3(obstacles(:,3),obstacles(:,2),obstacles(:,1),'y.')
    scrsz = get(0,'ScreenSize');
    set(gcf,'Position',[1 scrsz(2) scrsz(3) scrsz(4)])
    axis equal;view(3);rotate3d

    goalsfound=find(table((endN(:,3)-1)*matsize^2+(endN(:,2)-1)*matsize+endN(:,1))>0)';
    for i=1:size(goalsfound,2)
        %work backwards from the finish
        pathval(i).val=[endN(goalsfound(i),1),endN(goalsfound(i),2),endN(goalsfound(i),3)];
        maxval=0;
        % while the first step of the path is not at the start work backwards
        while pathval(i).val(1,1)~=startN(1) || pathval(i).val(1,2)~=startN(2) || pathval(i).val(1,3)~=startN(3)
            a=pathval(i).val(1,1);b=pathval(i).val(1,2);c=pathval(i).val(1,3);

            %find the next (actally previous) node
            nextnode=[a-1,b-1,c-1;a-1,b,c-1;a-1,b+1,c-1;
                      a,b-1,c-1;a,b,c-1;a,b+1,c-1;
                      a+1,b-1,c-1;a+1,b,c-1;a+1,b+1,c-1;

                      a-1,b-1,c;a-1,b,c;a-1,b+1,c;
                      a,b-1,c;a,b,c;a,b+1,c;
                      a+1,b-1,c;a+1,b,c;a+1,b+1,c;

                      a-1,b-1,c+1;a-1,b,c+1;a-1,b+1,c+1;
                      a,b-1,c+1;a,b,c+1;a,b+1,c+1;
                      a+1,b-1,c+1;a+1,b,c+1;a+1,b+1,c+1];
              
            nextnode=nextnode(nextnode(:,1)>0 & nextnode(:,1)<=matsize,:);
            nextnode=nextnode(nextnode(:,2)>0 & nextnode(:,2)<=matsize,:); 
            nextnode=nextnode(nextnode(:,3)>0 & nextnode(:,3)<=matsize,:); 
            %go through possible nextnodes and find the highest or the closest
            %if they are the same value
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
            %if there is no higher than the current pick a random one
            %(SHOULDN'T BE NEEDED)
%             if updated==false
%                 index=floor(rand*size(nextnode,1)+1);
%                 table(nextnode(index,1),nextnode(index,2));
%                 while table(nextnode(index,1),nextnode(index,2))==0;index=floor(rand*size(nextnode,1)+1);end
%             end

            pathval(i).val=[nextnode(index,:);pathval(i).val];

            %plot the pathval        
            if animate
                plot3(nextnode(index,3),nextnode(index,2),nextnode(index,1),'r.');
                drawnow;pause(0.05);
            end
            if takemovie
                plot3(nextnode(index,3),nextnode(index,2),nextnode(index,1),'r.');
                view([-53+index,38+i]);
                drawnow; 
                if isempty(F)
                    F=getframe; 
                else
                    F(end+1)=getframe;                     
                end
            end

        end
        plot3(pathval(i).val(:,3),pathval(i).val(:,2),pathval(i).val(:,1),'r');
    end   
    %show the path lines
    
    %show pathval at end
    if ~animate
        hold on
        for i=1:1:size(pathval,2)
            plot3(pathval(i).val(:,3),pathval(i).val(:,2),pathval(i).val(:,1),'r.');            
        end
    end
end

if takemovie
   keyboard
   movie(F);
   movie2avi(F,'3Dwavefrontmoveie.avi','compression','Cinepak','fps',30,'quality',100)
end


%% collsion checker
function [results]=checkcollision(J1,J2,J3)
keyboard
global workspace
newQ=[J1,J2,J3,0,0,0];

indexed_knowncoords=round(setdiff(workspace.knowncoords(GetImpLevInfo(workspace.knowncoords),:),workspace.indexedobsticles,'rows')/workspace.inc_size);
%this makes the check for a collision quicker
obsticle_points=workspace.indexedobsticles(GetImpLevInfo(workspace.indexedobsticles),:);
all_possible=round(workspace.unknowncoords(workspace.lev1unknown   ,:)/workspace.inc_size);
[nothing,index]=setdiff(all_possible,[indexed_knowncoords;obsticle_points],'rows');
unknown_points=workspace.unknowncoords(workspace.lev1unknown(index),:);

result=check_path_for_col(newQ,obsticle_points,unknown_points);

