%This function goes through and does all the test that are needed for the
%journal paper

function doKPItest(testnum,tests2do)

try    
close all

%% Check params passed in
if nargin<2
    if nargin<1
        testnum=6;
        display(['testnum being set to ', num2str(testnum)]);    
    end
    tests2do=[1,1,1,1,1,1];
    display(['Doing all test since you didnt say any different!']);
end

if length(tests2do)<6
    tests2do=[tests2do,zeros([1,6-length(tests2do)])];
    display(['You only passed a matrix of size ', length(tests2do), ' - make sure you pass it the size of number of KPIS (6)! Set other test to zero']);
end


%% Setup stuff
% globals
global workspace r scan

%The test cases Need to write this fucking shit down!!
if ~strcmp(pwd,'C:\MATLAB\R2007a\work\explorationMatlab\journal')
    try 
        cd C:\MATLAB\R2007a\work\explorationMatlab\journal
    catch
        display('fucking fix it!!');
    end
end

%run the exGUI at least once
if size(workspace,1)==0
    cd ..
    exGUI
    cd journal
end
















%% %%%%%
%|/     /|
%|\PI    |
%%%%%%% ^^^
%\subsubsection{KPI 1 - AXBAM Exploration vs Exhastive Search}
if tests2do(1)==1

%case study
%test number 6 has these times (from the text file
if testnum==6
    total_vpts=13;
    pp_time=[0.019128,0.015934, 0.0092056,0.37827,0.018423,1.839,0.46868,0.42561,0.010631,0.009863,0.57538,0.011844,1.2489,0.012443,0.012148,0.027637,0.011356, 0.011392,0.049364,0.012778,0.07671,0.012803,0.046118,0.013126];
    ex_time=[38.865782,22.936488,19.345931];
    tot_time=sum(pp_time)+sum(ex_time)
    av_time=tot_time/total_vpts;
    display(['Average time per decision= ',num2str(av_time)])
    maxAvailInfo=36688; %after 30 iterations
else
    av_time=inf;
    display(['Nothing given so : Average time per decision= ',num2str(av_time)])
end

%This is to give the stats of the 
load(['test',num2str(testnum),'state_data.mat']);
display(['Total time= ',num2str(state_data.time(end)/60),' minutes']);
display(['Percentage of total time= ',num2str(100*tot_time/(state_data.time(end))),'%']);

%now going through and doing exhastive search at each one of the search
%points
load(['test',num2str(testnum),'Xsearchdata.mat']);
load(['test',num2str(testnum),'robot_maxreach.mat']);
% lighting gouraud
% camlight
% for i=1:size(robot_maxreach.path,2)
%     plotdenso(r,robot_maxreach.path(i).all_steps(end,:))
%     i
%     pause    
% end

if testnum==6
    posefromRobMaxReach=[3,8,13];
    %so after a search is done then how many did AXBAM use before doing the
    %next search, so exhastive must do the same
    aftersearch_num_poses=[3,4,3];
else
    display('Need to look at the poses and find out where the scans were taken');
    error('define posefromRobMaxReach')
end
    
for i=1:size(Xsearchdata,2)
    global workspace Q
                
    %This makes the size of the search, cal with x^3*7*2
    numNBVanglesteps=[2,3,4];
    for searchsizeIndex=1:length(numNBVanglesteps)
        %find out where the arm is
        try    use_real_robot_GETJs(); 
        catch display('couldnt get joints from robot- updating the Q in memory regardless');
        end   
        %move to the correct place and load workspace
        workspace=Xsearchdata(i).workspace;
        %Set the supposed Q for here
        newQ=robot_maxreach.path(posefromRobMaxReach(i)).all_steps(end,:);
        valid=0;
        while valid==0
            [valid,all_steps]=pathplanner(newQ,false,true,true,100);            
            if valid==-1
                display('Im being told that this is a dangerous position when I know this is not the case-you have control, please use exGUI to move manually to the required newQ and remove selefscanning points al the way along')            
                keyboard
            end
        end
        try use_real_robot_MOVE(all_steps)
            use_real_robot_GETJs(); 
        catch display('couldnt move robot - updating the Q in memory regardless');
            %update the actual possion in memory in case it hasn't been done
            Q=newQ;
        end   
        
        %do appropriate search
        [data(i).totaltime(searchsizeIndex),data(i).searchspace(searchsizeIndex)]= near_exhastive_NBV_search(numNBVanglesteps(searchsizeIndex));    
        global bestviews
        currentBview=1;
        while ~isfield(bestviews,'valid') || bestviews(currentBview).valid==false            
            [bestviews(currentBview).valid,bestviews(currentBview).all_steps]=pathplanner(bestviews(currentBview).Q,false,true,true,30);            
            if ~bestviews(currentBview).valid
                display(['Couldnt find a path for best view num: ',num2str(currentBview)]);
                currentBview=currentBview+1;
            end
        end 

        display(['Found a path to best view num: ',num2str(currentBview)]);
        data(i).bvs(searchsizeIndex).bestviews=bestviews(currentBview:end);

        %EXPLORE
        nopathcount=0;
        for cur_scan=1:aftersearch_num_poses(searchsizeIndex)
            if cur_scan>1
                while data(i).bvs(searchsizeIndex).bestviews(cur_scan+nopathcount).valid==false            
                    [data(i).bvs(searchsizeIndex).bestviews(cur_scan+nopathcount).valid,...
                     data(i).bvs(searchsizeIndex).bestviews(cur_scan+nopathcount).all_steps]=...
                     pathplanner(data(i).bvs(searchsizeIndex).bestviews(cur_scan+nopathcount).Q,false,true,true,30);            
                    if ~data(i).bvs(searchsizeIndex).bestviews(cur_scan+nopathcount).valid
                        display(['Couldnt find a path for best view num: ',num2str(cur_scan+nopathcount)]);
                        nopathcount=nopathcount+1;
                    end
                end 
            end
            
            newQ=[data(i).bvs(searchsizeIndex).bestviews(cur_scan+nopathcount).Q(1:5),0];
            all_steps=data(i).bvs(searchsizeIndex).bestviews(cur_scan+nopathcount).all_steps;
            
            % Change newQ so if scanning on the downside (+ angles) -> start from max place and goes to the center, 
            % Alternately,it goes to +30', and then scan to the max negative possible
            qlimits=r.qlim; minimum_alpha=qlimits(5,1)*0.9;
            %determine the maximum angle
            if newQ(3)>scan.alpha_limited_condition; maximum_alpha=scan.alpha_limited; else maximum_alpha=scan.alpha;end    
            %determine where to start and where to tilt through too
            if scan.chosenview(3)>0
                newQ(5)=min(maximum_alpha,newQ(5)+scan.alpha/2);
                tilt_scan_range=minimum_alpha-newQ(5);
            else
                newQ(5)=max(minimum_alpha,newQ(5)-scan.alpha/2);
                tilt_scan_range=maximum_alpha-newQ(5);
            end

            %move through path
            try use_real_robot_MOVE([all_steps;newQ])
            catch display('could movent move');
            end

            %take a scan through determined tilt_scan_range
            try use_real_robot_SCAN(tilt_scan_range*180/pi);
                %update the latest position of robot
                use_real_robot_GETJs(); 
            catch display('couldnt scan or get updated position');
                Q=newQ+[0,0,0,0,tilt_scan_range,0];
            end          
               
            %this sort out the points that we have got from a scan
            try organise_data();
            catch display('cant do anything without actual scan data here');
            end
            data(i).size_allknown(searchsizeIndex).val(cur_scan)=size(workspace.knowncoords,1)+size(workspace.indexedobsticles,1);            
        end            
    end    
end

%save results to disk for later analysis
save('doKPItest.mat','data');

%need some way to compare the overall search with AXBAM, but AXBAM 
% at 3 possitions where we worked these out we have realistic exploration values 
% and we can compare these but the others are not recalculated so we can't compare them.
% 
% Fuck!
% So only compare the increase at these three positions with the exhastive search, but then the time is not correct so we could use the first (few value and compare these, 
% this will reduce the time and reduce the exhastive's ability to be the best but it would be a more realistic comparison
% , however we are
% 
% Can't include the first 2 poses since they weren't determined although they were path planned to and checked, 
% so they need to be included in both or in neither otherwise AXBAM looks quicker because it has additional decisions made for it
% Fuck!


keyboard


% 

    









end

%% %%%%%
%|/     ~|
%|\PI   /|\
%%%%%%% ^^
%\subsubsection{KPI 2 - Look Around}
if tests2do(2)==1
display('KPI2 is a visual inspection, just look at it!');

end

%% %%%%%
%|/     ~|
%|\PI   ~|
%%%%%%% ~
%\subsubsection{KPI 3 - Termination Conditions}
if tests2do(3)==1

if testnum==6
    maxAvailInfo=36688; %after 30 iterations
end

load(['test',num2str(testnum),'state_data.mat']);

display(['Termination at ',num2str((state_data.size_indexedobsticles(end)+state_data.size_known(end))/maxAvailInfo),'% of total']);


end

%% %%%%%
%|/    | |
%|\PI   ~|
%%%%%%%
%\subsubsection{KPI 4 - Safe Movement}
if tests2do(4)==1

    display('KPI4 - were there any collisions? If not then there is no problem!');
end

%% %%%%%
%|/     |~~
%|\PI    ~|
%%%%%%% ~~
%\subsubsection{KPI 5 - Map Quality}
if tests2do(5)==1
    
    
%     if testnum==6
%         display('Using the 9th mesh for test6 since same env, but improved map by steph');
%         load(['test',num2str(9),'hMesh.mat']);
%     else
        load(['test',num2str(testnum),'hMesh.mat']);
%     end

    
    figure
    bridgeSection;
    hold on;
    for i=1:10;
        [u(i),sig(i),D(i).vals]=comparemaps(i);
    end;
    figure(1)
    global r Q
    plotdenso(r,Q);
    camlight
    lighting gouraud
    axis equal;
    axis off;
    uiwait(msgbox('Move figure to the way you want then press OK'));
    figure(1)
    saveas(gcf,['test',num2str(testnum),'patches.png']);

    close all;
    
    figure;       
    bar(u);
    title('Mean over 10 planes')
    figure; 
    bar(sig)
    title('Standard deviation over 10 planes')

    figure
    names=[];vals=[];
    for i=1:size(D,2)
        names=[names;i*ones([size(D(i).vals,1),1])];
        %put all together and in mms (from meters originally)
        vals=[vals;D(i).vals(:)*1000];
    end
    boxplot(vals,names)
    uiwait(msgbox('Fig plot up with what you want then press OK'));
    saveas(gcf,['test',num2str(testnum),'boxplot.eps'])
    
    figure        
    index=GetImpLevInfo(hMeshdata.v);
    index2=find(hMeshdata.v(index,1)>-0.88);
    plot3(hMeshdata.v(index(index2),1),hMeshdata.v(index(index2),2),hMeshdata.v(index(index2),3),'r','marker','.','linestyle','none','markersize',0.1);axis equal; grid on
    
    



end


%% %%%%%
%|/     |~
%|\PI   |~|
%%%%%%%  ~
%\subsubsection{KPI 6 - C space opened up}
if tests2do(6)==1
    if testnum==6
        display('Using the 9th mesh for test6 since same env, but improved map by steph');
        load(['test',num2str(9),'hMesh.mat']);
    else
        load(['test',num2str(testnum),'hMesh.mat']);
    end




end


%% Overall catch statement
catch
    lasterr
    tempE=lasterror;
    display('question tempE stack for error details if above was not good enough.... you have control');
    keyboard
end
