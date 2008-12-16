%% pathplanner_water
%
% *Description:*  This is the hyper new plannner based upon water
% exploration, which is essentially a cource graph search with specific RRT
% characteristics to interconnect and finalise a real path 

%% Function Call
% 
% *Inputs:* 
%
% _newQ_ (many*6 double) radians - We can path plan to 1 or many destinations at once
%
% _animate_ (binary) If you want to see the print outs of the working
%
% _checkeachlink_ (binary) If you want to use pathplanner_new to plan between nodes
%
% _docolCheck_ (binary) only for testing purposes if we don't do collision checking on intermediate nodes
%
% *Returns:* 
%
% _pathval_ (struct) containing .result={-1,0,1} and .all_steps which is 6 collums of joints 

function [pathval] = pathplanner_water(newQ,animate,checkeachlink,docolCheck)

%% Input checks
if nargin<4
    docolCheck=true
    if nargin<3
        checkeachlink=false;
        if nargin<2
            animate=false;
            if nargin<1
                error('You must pass in at least a destination newQ')
            end
        end
    end
end

%% Variables

% close all;
global r Q workspace optimise graf_obs


% load('Journal/test9hMesh.mat')
% workspace.indexedobsticles=unique(workspace.inc_size*round(hMeshdata.v(GetimpLevInfo(hMeshdata.v),:)/workspace.inc_size),'rows');

%clear global graf_obs;

startN=Q(1,1:3);

qlimits=r.qlim;
%how many times bigger than the max move angle size can we accept
leaniancy=optimise.waterPPleaniancy;
matsize=floor((qlimits(1:3,2)-qlimits(1:3,1))./optimise.max_angle_for123'/leaniancy);
n=r.n;
L = r.link;
for piece=1:n   
    linkvals(piece).val=[L{piece}.alpha L{piece}.A L{piece}.D L{piece}.offset];   
end 

%% define workspace state: obstacle/unknown 
%this makes the check for a collision quicker
obsticle_points=workspace.indexedobsticles(GetImpLevInfo(workspace.indexedobsticles),:);

% If we include unknown
indexed_knowncoords=putInVoxels_gp(setdiff(workspace.knowncoords(GetImpLevInfo(workspace.knowncoords),:),workspace.indexedobsticles,'rows'),workspace.inc_size);
all_possible=putInVoxels_gp(workspace.unknowncoords(workspace.lev1unknown   ,:),workspace.inc_size);
[nothing,index]=setdiff(all_possible,[indexed_knowncoords;obsticle_points],'rows');
unknown_points=workspace.unknowncoords(workspace.lev1unknown(index),:);
% or else unknown is empty
% unknown_points=[];

%% check that the initial desired end joint config is safe and possible
validnewQ=true([size(newQ,1),1]);
for current_newQ=1:size(newQ,1)
    if ~check_path_for_col(newQ(current_newQ,:),obsticle_points,[],linkvals); 
        validnewQ(current_newQ)=false; 
        continue; 
    end
    % Check joint limits
    if ~isempty(find(newQ(current_newQ,:)'<qlimits(:,1), 1)) || ~isempty(find(newQ(current_newQ,:)'>qlimits(:,2), 1))
        validnewQ(current_newQ)=false; 
        continue;
    end
end

% if there are no valid one then return
if isempty(find(validnewQ==1,1))
    pathval(1).result=-1;
    pathval(1).all_steps=0;
    return
else
    pathval(1).result=0;
    pathval(1).all_steps=[];
end
%% Table and the size of the 3D graph
inc_size=((qlimits(1:3,2)-qlimits(1:3,1))./matsize)';
%set start to be 1
startN(1:3)=ceil((startN(1:3)-qlimits(1:3,1)')./inc_size);

% list of valid destinations
endN=newQ(validnewQ==1,1:3);
%update the list of newQ destinations
newQ=newQ(validnewQ==1,:);
for current_dest=1:size(endN,1)
    endN(current_dest,:)=ceil((endN(current_dest,:)-qlimits(1:3,1)')./inc_size);
end
%if we are at the absolute extremety of any joint angle then it will not so
%check and fix this
startN(startN==0)=1;
endN(endN==0)=1;

table=zeros([matsize(1),matsize(2),matsize(3)]);
table(startN(1),startN(2),startN(3))=1;

numofits=100;


%% IF we only have 1 destination check if we are at the end (exit if we are)
if size(newQ,1)==1
    if isempty(find(newQ~=Q, 1))      
        pathval(1).result=1;
        return;
    %if the only differnce is the last 3 joints do old way 
    elseif isempty(find(newQ(1:3)~=Q(1:3), 1))
        [pathval(1).result,pathval(1).all_steps] = pathplanner_new(newQ,tryalternate,check_arm_perms,useMiddleQ2,numofPPiterations,0);
        return
    end
end


%% start the figures
if animate
    close all
    figure(1)
    subplot(1,2,2);camlight;axis equal;hold on;view(3)
    plot3(obsticle_points(:,1),obsticle_points(:,2),obsticle_points(:,3),'marker','.','Color',[.2,.2,.1],'linestyle','none');
    % plot3(unknown_points(:,1),unknown_points(:,2),unknown_points(:,3),'marker','.','Color',[0.9,0.9,0.9],'linestyle','none');
    subplot(1,2,1);
    xlabel('Joint 1 Increments');ylabel('Joint 2 Increments');zlabel('Joint 3 Increments')
    axis([1 matsize(1) 1 matsize(2) 0 matsize(3)]);
    view(3)
end


%% Make config space obstacles
tic

update_jointconfig_obs(matsize,obsticle_points,unknown_points);

toc

%% Release water from start
tic
for itnum = 1:numofits; 
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
        if animate; display('Not all paths are possible');end
        break; 
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
    table((graf_obs(:,3)-1)*matsize(1)*matsize(2)+(graf_obs(:,2)-1)*matsize(1)+graf_obs(:,1))=-inf;
    
    %if we have got to the destination then break - a path is possible
    if isempty(find(table((endN(:,3)-1)*matsize(1)*matsize(2)+(endN(:,2)-1)*matsize(1)+endN(:,1))==0,1)); 
        break; 
        if animate; display('found solution'); end
            
    end
end
toc
% % keyboard
%% Go from destination to source (ANIMATE)
% 
%if there is a way to get to the start
if ~isempty(find(table((endN(:,3)-1)*matsize(1)*matsize(2)+(endN(:,2)-1)*matsize(1)+endN(:,1))>0,1))
    %plot start and finish
    if animate        
        hold on;
        plot3(startN(3),startN(2),startN(1),'r*')
        plot3(endN(:,3),endN(:,2),endN(:,1),'g*')
        plot3(graf_obs(:,3),graf_obs(:,2),graf_obs(:,1),'y.')
        scrsz = get(0,'ScreenSize');
        set(gcf,'Position',[1 scrsz(2) scrsz(3) scrsz(4)])
        axis equal;view(3);rotate3d
    end

    goalsfound=find(table((endN(:,3)-1)*matsize(1)*matsize(2)+(endN(:,2)-1)*matsize(1)+endN(:,1))>0)';

    display(['Using only graph search, goals found =', num2str(size(goalsfound,2))]);
    
%     for cur_goal=1:size(goalsfound,2) 
    %we want these to line up with the endN which also align with newQ
    for cur_goal=1:size(endN,1)
        pathval(cur_goal).newQ=newQ(cur_goal,:);
        %if goal is found
        if ~isempty(find(table((endN(cur_goal,3)-1)*matsize(1)*matsize(2)+(endN(cur_goal,2)-1)*matsize(1)+endN(cur_goal,1))>0,1))            
            %work backwards from the finish
            pathval(cur_goal).val=[endN(cur_goal,1),endN(cur_goal,2),endN(cur_goal,3)];
            pathval(cur_goal).val=[endN(cur_goal,1),endN(cur_goal,2),endN(cur_goal,3)];
            maxval=0;
            % while the first step of the path is not at the start work backwards
            while pathval(cur_goal).val(1,1)~=startN(1) || pathval(cur_goal).val(1,2)~=startN(2) || pathval(cur_goal).val(1,3)~=startN(3)
                a=pathval(cur_goal).val(1,1);b=pathval(cur_goal).val(1,2);c=pathval(cur_goal).val(1,3);

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
                %not equal any in the current path
                if size(pathval(cur_goal).val,1)>1
%                     nextnode=setdiff(nextnode,pathval(cur_goal).val,'rows');
                    for cur_path_step=1:size(pathval(cur_goal).val,1)
                        nextnode=nextnode(nextnode(:,1)~=pathval(cur_goal).val(cur_path_step,1) | ...
                                          nextnode(:,2)~=pathval(cur_goal).val(cur_path_step,2) | ...
                                          nextnode(:,3)~=pathval(cur_goal).val(cur_path_step,3),:);
                    end
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

                pathval(cur_goal).val=[nextnode(index,:);pathval(cur_goal).val];

                %plot the pathval        
                if animate
                    plot3(nextnode(index,3),nextnode(index,2),nextnode(index,1),'r.');
                    drawnow;pause(0.05);
                end
            end
        
        
%% check the inbetween nodes of the path
            if ~checkeachlink
                %this is the next int after the leaniancy so we can use for
                %indexing
                leanINT=ceil(optimise.waterPPleaniancy);
                
                %this prealocates the size for all steps and sets first val
                %to be the currnt Q               
                pathval(cur_goal).all_steps=[Q;...
                                            zeros([(size(pathval(cur_goal).val,1)-1)*leanINT-1,6])];
                currentpathtocheck=zeros([(leanINT-1)*size(pathval(cur_goal).val,1),6]);
                inbetweensteps=zeros([leanINT-1,6]);
                for pnode=1:size(pathval(cur_goal).val,1)
                    %determine the latest node added to the list
                    currentQnode=pathval(cur_goal).all_steps(leanINT*(pnode-1)+1,:);
                    
                    %determine the next node in terms of Q
                    [J1,J2,J3]=mapindextojoints(pathval(cur_goal).val(pnode,1),...
                                      pathval(cur_goal).val(pnode,2),...
                                      pathval(cur_goal).val(pnode,3),qlimits,matsize);
                    nextQnode=[J1,J2,J3,...
                               Q(4:6)+pnode*(newQ(cur_goal,4:6)-Q(4:6))/size(pathval(cur_goal).val,1)];
                    %go through the added increments and keep these in between steps 
                    for curr_step=1:leanINT-1
                        ratiothrough=curr_step*optimise.waterPPleaniancy/leanINT;
                        inbetweensteps(curr_step,:)=[currentQnode+(nextQnode-currentQnode)*...
                                                     ratiothrough/optimise.waterPPleaniancy];
                        
                    end
                    %this stickt together previous nodes, middle nodes and the next node
                    pathval(cur_goal).all_steps(leanINT*pnode-(leanINT-2):leanINT*pnode,:)=inbetweensteps;
                    pathval(cur_goal).all_steps(leanINT*pnode+1,:)=nextQnode;
                    
                    %keep the in between Q vals that need to be checked
                    currentpathtocheck((leanINT-1)*(pnode-1)+1:(leanINT-1)*pnode,:)=inbetweensteps;
                    
                end

                %this tacs on the last, actual destination, pose
                currentQnode=pathval(cur_goal).all_steps(end,:);
                nextQnode=newQ(cur_goal,:);
                %go through the added increments and keep these in between steps 
                for curr_step=1:leanINT-1
                    ratiothrough=curr_step*optimise.waterPPleaniancy/leanINT;
                    inbetweensteps(curr_step,:)=[currentQnode+(nextQnode-currentQnode)*...
                                                 ratiothrough/optimise.waterPPleaniancy];

                end
                pathval(cur_goal).all_steps=[pathval(cur_goal).all_steps;inbetweensteps;newQ(cur_goal,:)];
                %check the inbetween nodes of this step too
                currentpathtocheck=[currentpathtocheck;inbetweensteps];
                
                
                %we only need to check every other than the first and last
                %value
                if docolCheck
                    [pathval(cur_goal).result,pathval(cur_goal).unknown_points_result]=check_path_for_col(currentpathtocheck,obsticle_points,unknown_points,linkvals);
                else %%NOTE: ONLY FOR TESTING PURPOSES SHOULD WE EVER GET HERE
                    pathval(cur_goal).result=1;
                    pathval(cur_goal).unknown_points_result=[];
                end
            else
                pathval(cur_goal).all_steps=Q;
                currQ=Q; if animate;  display('Danger! Dont quit here since global Q value is currently lost'); end
                for pnode=1:size(pathval(cur_goal).val,1)-1                
                    [J1,J2,J3]=mapindextojoints(pathval(cur_goal).val(pnode,1),pathval(cur_goal).val(pnode,2),pathval(cur_goal).val(pnode,3),qlimits,matsize);
    %                     startQnode=[J1,J2,J3,Q(4:6)+pnode*(newQ(cur_goal,4:6)-Q(4:6))/size(pathval(cur_goal).val,1)];
                    Q=[J1,J2,J3,Q(4:6)+pnode*(newQ(cur_goal,4:6)-Q(4:6))/size(pathval(cur_goal).val,1)];

                    [J1,J2,J3]=mapindextojoints(pathval(cur_goal).val(pnode+1,1),pathval(cur_goal).val(pnode+1,2),pathval(cur_goal).val(pnode+1,3),qlimits,matsize);
                    % at pnode+1
                    nextQnode=[J1,J2,J3,Q(4:6)+(pnode+1)*(newQ(cur_goal,4:6)-Q(4:6))/size(pathval(cur_goal).val,1)];
    %                 tempsteps=startQnode;
    %                 for curr_step=1:optimise.waterPPleaniancy
    %                     tempsteps=[tempsteps;...
    %                                Q+(nextQnode-Q)*curr_step/optimise.waterPPleaniancy];
    %                 end
    %                 [result,unknown_points_result]=check_path_for_col(tempsteps,obsticle_points,unknown_points);

                    %use original path planer to get between nodes, if this is
                    %not possible you should use the graphs search again
                    [result,tempsteps]=pathplanner_new(nextQnode,0,1,0,0,0);



                    if result==true
    %                     if unknown_points_result==false
    %                         display('No actual collsion detected but moving through unknown space');
    %                     end
                        pathval(cur_goal).all_steps=[pathval(cur_goal).all_steps;tempsteps];
                        %this should be moved somewhere else so it isn't
                        %continuously defined
                        pathval(cur_goal).result=1;
                    else
                        pathval(cur_goal).result=0;
                        if animate display(['Cant get to the required newQ: ',num2str(cur_goal),', between 2 links-so breaking']);end
                        break;
                    end
                end
                Q=currQ; 
                if animate; display('Ok to do anything now');end
            end

        
        
        
            %show the path lines
            if animate
                plot3(pathval(cur_goal).val(:,3),pathval(cur_goal).val(:,2),pathval(cur_goal).val(:,1),'r');
                plot3(pathval(cur_goal).val(:,3),pathval(cur_goal).val(:,2),pathval(cur_goal).val(:,1),'r');

                %Plot the robot path
                [J1,J2,J3]=mapindextojoints(pathval(cur_goal).val(:,1),pathval(cur_goal).val(:,2),pathval(cur_goal).val(:,3),qlimits,matsize);
                subplot(1,2,2);
                demopath_new([J1,J2,J3,zeros(size(J3,1),1),zeros(size(J3,1),1),zeros(size(J3,1),1)])
                subplot(1,2,1);
            end
        else
            if animate
                display('No paths found in graph search');
            end
        end
    end    
end