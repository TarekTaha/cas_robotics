%% function testpathplanner(numtests)
% Description: This function is used to find valid start and end points for
% a certain amount of tests (numtests). It then finds paths for each on of
% these if they are possible. If they are not possible then 

function testpathplanner(numtests,useMiddleQ2,check_arm_perms,makenewgoals,useDijkstra,waterplanner,groupplan)
global Q r densoobj workspace optimise

n = r.n;
L = r.link;
t = r.base;
qlimits=r.qlim;

%% going through and setting up the default variables
if nargin<7
    groupplan=false;
    if nargin<6
        waterplanner=true;
        if nargin<5
            useDijkstra=false;
            if nargin<4
                makenewgoals=true;
                if nargin<3   
                    check_arm_perms=true;
                    if nargin<2   
                        useMiddleQ2=true;
                        if nargin==0
                            numtests=1000;
                            display(strcat('Setting number of tests to:',num2str(numtests),' since nothing was passed'));
end; end; end; end; end; end; end

%can delete if dijkstra is not used
if useDijkstra; dijkstracounter=0;end 

%if we don't want to make new goals we need to load them, if not possible
%we still need to make them
if ~makenewgoals
    try load pathstartNend.mat;
        if size(startQs,1)~=numtests; 
            display('Error; Not correct num of goals in loaded file, making up new goals');
            makenewgoals=1;        
        end
    catch display('Error; Loading file pathstartNend.mat-getting new ones');
        makenewgoals=1;        
    end
end

%% all invalid start or finish will be saved here, then these will be removed
if makenewgoals
    %make up some end points
    startQs=[Q];
    endQs=[];
    tempendQs=rand(numtests,6);
    for i=1:size(tempendQs,2)
        endQs=[endQs,tempendQs(:,i)*(-qlimits(i,1)+qlimits(i,2))+qlimits(i,1)];
    end

    notvalid=[]; 
    %go through and remove poses which are impossible
    for test=1:numtests
        %check ends
        notvalid=true;
        while notvalid
            notvalid=false;
            t = r.base;
            ends_to_test=endQs(test,:);   

            %get pos of the end effector point at the end
            endPoint=fkine(r,ends_to_test);        
            %if not within the workspace of robot
            if isempty(GetImpLevInfo(endPoint(1:3,4)'))
                notvalid=true;
            end
            %we only want points above z=ground
%             if endPoint(3,4)<0
%                 notvalid=true;
%             end
            notvalid=~check_path_for_col(ends_to_test);

%             %go through and check each piece for a collision
%             for piece=1:n
%                 t = t * L{piece}(ends_to_test(piece));
%                 %check ellipses 2->7
%                 if ~check_FF(t,densoobj(piece+1).ellipse,workspace.indexedobsticles) 
%                     notvalid=true;
%                 end
%                 %check that the point is not inside the first two ellipses
%                 if piece<3 && ~check_FF(t,densoobj(piece).ellipse,[endPoint(1,4),endPoint(2,4),endPoint(3,4)])
%                     notvalid=true;
%                 end
                %make up another end point since previous one will not happen
                if notvalid
                    tempendQs=rand(1,6);
                    for i=1:size(tempendQs,2)
                        endQs(test,i)=tempendQs(:,i)*(-qlimits(i,1)+qlimits(i,2))+qlimits(i,1);
                    end                
%                     break; 
                end
%             end
        end   
    end    

    %now set all start points to be the same as the previous end point
    %except the the first which is the current Q
    startQs=[startQs;endQs(1:end-1,:)];
    save('pathstartNend.mat','startQs','endQs');
end

%% try and actually get paths
starttime=clock;    
allpathsfound=[];
%this will count num of alt paths found with pose selection change
numofaltpathsused=0;
pathsfound_counter=1;
pathdata=[];
impossiblepathcnt=0;
if groupplan
    tic
    path_val=pathplanner_water(endQs,false,false);
    toc
    allpathsfound=[];
    %go through the results and check how many are valid
    for i=1:numtests
        if path_val(i).result==true
            allpathsfound=[allpathsfound;path_val(i).result];
        else  
            impossiblepathcnt=impossiblepathcnt+1;
        end
    end 
    save('path_val.mat','path_val');
else
    for i=1:numtests
        Q=startQs(i,:);
        newQ=endQs(i,:);
        if waterplanner==true
          profile clear; profile on
            path_val=pathplanner_water(newQ,false);
            profile off; profile viewer
            keyboard
            temppathfound=path_val(1).result;
            all_steps=path_val(1).all_steps;
        else
            [temppathfound,all_steps]=pathplanner(newQ,check_arm_perms,useMiddleQ2,optimise.numofPPiterations,false);
        end

        %if we want to try and get a path with dijkstra algorithm
        if ~temppathfound && useDijkstra
            [temppathfound,all_steps]=practiceDijkstra(startQ,endQ);
            if temppathfound==1
                all_steps=all_steps.alljoints;
                dijkstracounter=dijkstracounter+1;
            end
        end

        if temppathfound==1
            if (all_steps(end,:)-newQ~=0)
                numofaltpathsused=numofaltpathsused+1;
                %we have to make the new start different because we found an
                %alternate solution
                if i<size(startQs,1)
                    startQs(i+1,:)=all_steps(end,:);
                end
            end
            %keep the path that was taken
            pathdata(pathsfound_counter).all_steps=all_steps;
            pathsfound_counter=pathsfound_counter+1;
        else
            %Since we didn't get to the last end the next start Q is equal to
            %the current start Q, as long as we are not at the end
            if i<size(startQs,1)
                startQs(i+1,:)=startQs(i,:);             
                %display(strcat('For path num:', num2str(i+1),', we set the startQ to be the same as previous startQ since no path found'));
            end
            if temppathfound==-1
                impossiblepathcnt=impossiblepathcnt+1;
            end
        end
        allpathsfound=[allpathsfound;temppathfound];

    end
end
%% Display Results
%clc
finshtime=etime(clock,starttime);
display('0-0-0-0-0-0-0-0-0-0-0-0-0-0-0-0-0-0');
display(strcat('Statistics........ for: ',num2str(numtests),'tests, took: ',num2str(finshtime),' seconds'));
if waterplanner==true; display('Using the waterplanner'); end
display(['Num of valid paths found = ',num2str(length(find(allpathsfound==1))), ' at ',num2str(finshtime/length(find(allpathsfound==1))),' sec/path']);
display(strcat('Num of impossible end posses = ',num2str(impossiblepathcnt)));
display(strcat('..of which No. alternate paths = ',num2str(numofaltpathsused)));
% display(strcat('Num of times no path found = ',num2str(length(find(allpathsfound==0)))));
% display('Saving all steps of all paths to file pathdata.mat');
display(strcat('Variables are as follows{useMiddleQ2=',num2str(useMiddleQ2),...
                '; check_arm_perms=',num2str(check_arm_perms)));
display('..........................................................................................');
save('pathdata.mat','pathdata');