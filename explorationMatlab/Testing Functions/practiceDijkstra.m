%% Function practiceDijkstra
% A return of pathfound==0 means there is no actual path (no links to dest)
% of there is no intermediate path between nodes because of collisions
% if pathfound==-1 it means that there was a section of path found by
% dijkstra whereby it is not possilbe ot move to these nodes (in the path)
% because of collsions, they are impossible final paths.
% pathdata will hold all the joints between start and finish including all
% intermediate paths
function [pathfound,pathdata]=practiceDijkstra(startQ,endQ,numofpaths,DISPON)

global r Q workspace optimise

% Make sure indexedobsticles are only within reach (minimise collsion checking)
workspace.indexedobsticles=workspace.indexedobsticles(GetImpLevInfo(workspace.indexedobsticles),:);

% this is the points that are going to be used to traverse through
display('Note: You using the known points as your nodes - got from workspace.mat variable (different to pointdata)!!!!')
knownindex=workspace.knowncoords(GetImpLevInfo(workspace.knowncoords),:);
knownindex=setdiff(knownindex,workspace.indexedobsticles,'rows');
% get rid of the points around the feotal positon robot bottom (which is known to be safe
index=find((knownindex(:,1)<workspace.robotsize(1,2) & knownindex(:,1)>workspace.robotsize(1,1)) &...
           (knownindex(:,2)<workspace.robotsize(2,2) & knownindex(:,2)>workspace.robotsize(2,1)) &...
            knownindex(:,3)<workspace.robotsize(3,2));
newindex=setdiff([1:size(knownindex,1)]',index);
%nodes are numbers and are calculated by dividing by the inc size of workspace 
nodes=knownindex(newindex,:)/workspace.inc_size;
noOfNodes=size(nodes,1);

%try and work out where the source node is based upon current joints, if not known then get closest known point to it
tr=fkine(r,startQ);
sourcepoint=round(tr(1:3,4)/workspace.inc_size);
s=find(nodes(:,1)==sourcepoint(1) & nodes(:,2)==sourcepoint(2) & nodes(:,3)==sourcepoint(3));
if size(s,1)==0
    [unneededval,s]=min(sqrt((nodes(:,1)-sourcepoint(1)).^2+(nodes(:,2)-sourcepoint(2)).^2+(nodes(:,3)-sourcepoint(3)).^2));
end

tr=fkine(r,endQ);
destpoint=round(tr(1:3,4)/workspace.inc_size);
d=find(nodes(:,1)==destpoint(1) & nodes(:,2)==destpoint(2) & nodes(:,3)==destpoint(3));
if size(d,1)==0
    [unneededval,s]=min(sqrt((nodes(:,1)-destpoint(1)).^2+(nodes(:,2)-destpoint(2)).^2+(nodes(:,3)-destpoint(3)).^2));
end

%% Check nargin
if nargin<4
    DISPON=false;
    if nargin<3
        numofpaths=1;
        if nargin==0 %if we are passed nothing then we make up the start and destination            
            d=ceil(rand()*noOfNodes);
            if DISPON
                close all;
                plot3(workspace.indexedobsticles(:,1),workspace.indexedobsticles(:,2),workspace.indexedobsticles(:,3),'.r');hold on;
                plot3(knownindex(:,1),knownindex(:,2),knownindex(:,3),'.');
            end
        end
    end
end

%% make up link table
%should increase the weight of the links to the nodes which are adjacent to
%obstacles
if isfield(workspace,'linktable') && noOfNodes==size(workspace.linktable,1)
    linktable=workspace.linktable;
else
    linktable=ones([noOfNodes,noOfNodes])*inf;
    for i=1:noOfNodes
        index=find(round(abs(nodes(i:end,1)-nodes(i,1))+abs(nodes(i:end,2)-nodes(i,2))+abs(nodes(i:end,3)-nodes(i,3)))==1);
        index=index+(i-1);
        linktable(i,index)=1;
        linktable(index,i)=1;
    end
    workspace.linktable=linktable;
end 

%% Set other variables
totalpath=[];
startandend=[];
alljoints=[];
totaltime=clock;
if DISPON plotdata=[];end
qlimits=r.qlim;
pathfound=1;
pathdata=[];
failurecount=0;


%% go through a number of paths
for i=1:numofpaths 
%     starttime=clock; 
    [path, totalCost]=dijkstra_new(noOfNodes, linktable, s, d);
%     display(strcat('To go between :',num2str(s),'=(',num2str(knownindex(s,:)),') and: ',num2str(d),'= (',num2str(knownindex(d,:)),')_Took:',num2str(etime(clock,starttime))));display(strcat('The path is nodes:',NUM2STR(path)));display(num2str(newestpath));
    if size(path,1)==0
        pathfound=0;
        display('FAILURE: no path found: if numofpaths==1 then we quit anyway, else making new dest');
        d=ceil(rand()*noOfNodes);
        failurecount=failurecount+1;
        continue;
    end
    %take out new path 
    newestpath=knownindex(path,:);
    newstartandend=[knownindex(s,:),knownindex(d,:)];

    if DISPON 
        %Plot new path plot data: first delete old path plot data
        for j=1:length(plotdata); try delete(plotdata(j));end;end; plotdata=[];    
        plotdata=[plotdata,plot3(newstartandend(:,1),newstartandend(:,2),newstartandend(:,3),'b*')];
        hold on;    plotdata=[plotdata,plot3(newestpath(:,1),newestpath(:,2),newestpath(:,3),'r')];drawnow;
    end

    %go through each step of the path and move to this position
    for j=1:size(path,2)
        pt=knownindex(path(j),:);
        dist=[inf,inf,inf,inf];
        %1st of 3 ways to get the pose - guess Q at Q
        Q1=streamOnto_mine_nodirection(r, pt, Q);
        dist(1)=dist_pt2tr(pt,fkine(r,Q1));
        if dist(1)>optimise.minAccepDis
            %2nd of 3 ways to get the pose - guess Q at 0
            Q2=streamOnto_mine_nodirection(r, pt);            
            dist(2)=dist_pt2tr(pt,fkine(r,Q2));
            if dist(2)>optimise.minAccepDis
                %3rd of 3 ways to get the pose - guess Q at pi away from Q
                Q_guess=Q+((sqrt(Q.^2)~=Q)*2-1)*pi;
                Q3=streamOnto_mine_nodirection(r, pt, Q_guess);
                dist(3)=dist_pt2tr(pt,fkine(r,Q3));               
                if dist(3)>optimise.minAccepDis                    
                    Q_guess2=[];
                    for current_j=1:size(qlimits,1)
                        Q_guess2=[Q_guess2,rand()*(-qlimits(current_j,1)+qlimits(current_j,2))+qlimits(current_j,1)];
                    end
                    Q4=streamOnto_mine_nodirection(r, pt, Q_guess2);
                    dist(4)=dist_pt2tr(pt,fkine(r,Q4));
                end
            end
        end        
        
        [nothing,index]=min(dist);
        switch(index)
            case(1) 
                newQ=Q1';
            case(2) 
                newQ=Q2';
            case(3) 
                newQ=Q3';
            case(4) 
                newQ=Q4';
        end
        [pathfound,all_steps] = pathplanner(newQ);
        %we can't have it returning no path
        if (pathfound==0)         
            display('no intermediate path found between steps-trying to get to next step');
            continue
        elseif (pathfound==-1) 
            continue; %go to next point in the path and try and get there
        end
        
        %it won't get here unless there is a valid path
        if size(all_steps,1)>0 && DISPON 
             demopath_new(all_steps);             
        end
        %update latest actual Q and save in alljoints of path
        Q=newQ;
        alljoints=[alljoints;all_steps];
        
        if DISPON 
            plotdenso(r,Q,false,false);
            drawnow;axis([-1 1 -1 1 0 1.5]);
        end
        
    end

    if pathfound~=1 %then we have not actually reached end point so it will jump
        display('FAILURE: Have not reach end of path - there is no path so setting new dest');
        d=ceil(rand()*noOfNodes);
        failurecount=failurecount+1;
        continue;
    end
        
    totalpath=[totalpath;newestpath];
    startandend=[startandend;newstartandend];
    
    %this is only used if there are more than one path requested, here we make the destination a random number!
    s=d;
    d=ceil(rand()*noOfNodes);
end 

%% Save all data to var to output
pathdata.startandend=startandend;
pathdata.totalpath=totalpath;
pathdata.totaltime=etime(clock,totaltime);
pathdata.alljoints=alljoints;
pathdata.failurecount=failurecount;
%save('prac_dijk.mat','pathdata');

% %% Plotting
% %delete old path plot data
% for j=1:length(plotdata); try delete(plotdata(j));end;end;
% 
% 
% %Plot new path plot data
% plot3(prac_dijk.startandend(:,1),prac_dijk.startandend(:,2),prac_dijk.startandend(:,3),'b*');
% hold on;    plot3(prac_dijk.totalpath(:,1),prac_dijk.totalpath(:,2),prac_dijk.totalpath(:,3),'r');
% hold on;    toplot=setdiff(knownindex(1:maxpnt,:),prac_dijk.startandend(:,1:3),'rows');
% plot3(toplot(:,1),toplot(:,2),toplot(:,3),'.','Color',[0.9,0.9,0.9]);
% 
% demopath_new(prac_dijk.alljoints);
