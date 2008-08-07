function exploreWaterpump()
close all;

takemovie=true;
animate=true;
numdests=10;

matsize=200; %200 is good (length of 1 side)
table=zeros([matsize,matsize]);
numofits=10000;

% startN=round(rand(1,2)*matsize);
startN=[matsize/2,matsize/2];
endN=[ceil(rand(numdests-1,2)*matsize);matsize/2,matsize];

%make sure they are different and not zero
while startN(1)==0 || startN(2)==0; startN=round(rand(1,2)*100); end
% while (endN(1)==startN(1) && endN(2)==startN(2)) || endN(1)==0 || endN(2)==0;  endN=round(rand(1,2)*100); end

%% Make obstacles
obstacles=[];

%spiral
% obdensity=8;
% t = 0:0.1:obdensity*matsize; obstacles=unique(round([([t .* cos(t)]'+obdensity*matsize)./obdensity, ([t .* sin(t)]'+obdensity*matsize)./obdensity]),'rows');

%random stalagmite & stalactite like obstacles
for i=1:4:matsize
    if rand>0.85
        barlength=round(rand*matsize);while barlength==matsize; round(rand*matsize);end
        obstacles=[obstacles;[[1:matsize-barlength]',i*ones([matsize-barlength,1])]];
    end
end

for i=3:4:matsize
    if rand>0.85
        barlength=round(rand*matsize);while barlength==matsize; round(rand*matsize);end
        obstacles=[obstacles;[[barlength+1:matsize+1]',i*ones([matsize-barlength+1,1])]];
    end
end

% random dust
obstacles=[obstacles;round(rand(matsize,2)*matsize)];

if ~isempty(find(obstacles(:,1)==startN(1) & obstacles(:,2)==startN(2),1))
    badnode=find(obstacles(:,1)==startN(1) & obstacles(:,2)==startN(2));
    obstacles=obstacles([1:badnode-1,badnode+1:end],:);
end
if ~isempty(find(obstacles(:,1)==endN(1) & obstacles(:,2)==endN(2),1))
    badnode=find(obstacles(:,1)==endN(1) & obstacles(:,2)==endN(2));
    obstacles=obstacles([1:badnode-1,badnode+1:end],:);
end
%get rid of zeros
obstacles=obstacles(obstacles(:,1)>0 & obstacles(:,2)>0 & obstacles(:,1)<=matsize & obstacles(:,2)<=matsize,:);
% plot(obstacles(:,1),obstacles(:,2),'r.');

%set start to be 1
table(startN(1),startN(2))=1;

%% Release water from start (wavefront search)
tic
for i=1:numofits;
    [a,b]=find(table==1);
%    if there are none ==1 then we have explored as much as possible
    if isempty(a); display('No path possible');break; end
    %set to update the surrounding cells
    toupdate=[a-1,b-1;a-1,b;a-1,b+1;
              a,b-1;a,b;a,b+1;
              a+1,b-1;a+1,b;a+1,b+1];
          toupdate=toupdate(toupdate(:,1)>0 & toupdate(:,1)<matsize+1,:);
          toupdate=toupdate(toupdate(:,2)>0 & toupdate(:,2)<matsize+1,:);
          
    %find all other cells which are greater than 0
    [c,d]=find(table>1);
    
    %put together into a single index and only use unique
    singleindex=unique([(toupdate(:,2)-1)*matsize+toupdate(:,1);(d-1)*matsize+c]);
    %only update each cell once         
    %toupdate=unique([toupdate;[c,d]],'rows');
    
    table(singleindex)=table(singleindex)+1;
    
    %remove obstacles (set to 0)
    table((obstacles(:,2)-1)*matsize+obstacles(:,1))=0;
    
    %if we have got to the destination then break - a path is possible
    if isempty(find(table((endN(:,2)-1)*matsize+endN(:,1))==0,1)); break; end
    %draw every now and again
    if rand>0.97 && animate==true; imshow(table/max(max(table)));drawnow;end
    if takemovie && mod(i,2)==0 imshow(table/max(max(table))); F(i/2)=getframe; end
end
toc

%% Go from destination to source (ANIMATE)... 

%if there is a way to get to the start
if ~isempty(find(table((endN(:,2)-1)*matsize+endN(:,1))>0,1))
    %plot start and finish
    imshow(table/max(max(table)));   
    hold on;
    plot(startN(2),startN(1),'r*')
    plot(endN(:,2),endN(:,1),'g*')
    plot(obstacles(:,2),obstacles(:,1),'y.')
    scrsz = get(0,'ScreenSize');
    set(gcf,'Position',[1 scrsz(2) scrsz(3) scrsz(4)])
    
    goalsfound=find(table((endN(:,2)-1)*matsize+endN(:,1))>0)';
    for i=1:size(goalsfound,2)
        %work backwards from the finish
        pathval(i).val=[endN(goalsfound(i),1),endN(goalsfound(i),2)];
        maxval=0;
        % while the first step of the path is not at the start work backwards
        while pathval(i).val(1,1)~=startN(1) || pathval(i).val(1,2)~=startN(2)
            a=pathval(i).val(1,1);b=pathval(i).val(1,2);

            %find the next (actally previous) node
            nextnode=[a-1,b-1;a-1,b;a-1,b+1;
                      a,b-1;a,b;a,b+1;
                      a+1,b-1;a+1,b;a+1,b+1];
            nextnode=nextnode(nextnode(:,1)>0 & nextnode(:,1)<=matsize,:);
            nextnode=nextnode(nextnode(:,2)>0 & nextnode(:,2)<=matsize,:); 
            %go through possible nextnodes and find the highest or the closest
            %if they are the same value
            index=1;updated=false;
            for j=1:size(nextnode,1)
                if table(nextnode(j,1),nextnode(j,2))>maxval 
                    index=j;maxval=table(nextnode(j,1),nextnode(j,2));
                    updated=true;
                elseif table(nextnode(j,1),nextnode(j,2))==maxval &&...
                        ((startN(1)-nextnode(j,1))^2+(startN(2)-nextnode(j,2))^2)<=...
                        ((startN(1)-nextnode(index,1))^2+(startN(2)-nextnode(index,2))^2)
                    index=j;maxval=table(nextnode(j,1),nextnode(j,2));
                    updated=true;
                end
            end
            %if there is no higher than the current pick a random one
            %(SHOULDN'T BE NEEDED)
            if updated==false
                index=floor(rand*size(nextnode,1)+1);
                table(nextnode(index,1),nextnode(index,2));
                while table(nextnode(index,1),nextnode(index,2))==0;index=floor(rand*size(nextnode,1)+1);end
            end

            pathval(i).val=[nextnode(index,:);pathval(i).val];

            %plot the pathval        
            if animate
                plot(nextnode(index,2),nextnode(index,1),'r.');
                drawnow;pause(0.05);
            end
            if takemovie
                plot(nextnode(index,2),nextnode(index,1),'r.');
                drawnow; 
                F(end+1)=getframe; 
            end
            
        end
    end   
    %show pathval at end
    if ~animate
        hold on
        for i=1:1:size(pathval,2)
            plot(pathval(i).val(:,2),pathval(i).val(:,1),'r.');
        end
    end
end

if takemovie
   keyboard
   movie(F);
   movie2avi(F,'2Dwavefrontmoveie.avi','compression','Cinepak','fps',30,'quality',100)
end