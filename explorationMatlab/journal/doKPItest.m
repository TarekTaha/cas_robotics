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
    display(['You only passed a matrix of size ', num2str(length(tests2do)), ' - make sure you pass it the size of number of KPIS (6)! Set other test to zero']);
end


%% Setup stuff
% globals
global workspace r scan

%The test cases Need to write this fucking shit down!!
if ~strcmp(pwd,[matlabroot,'\work\explorationMatlab\journal'])
    try 
        cd ([matlabroot,'\work\explorationMatlab\journal']);
    catch
        try cd ([matlabroot,'\work\Gavin\PhD\explorationMatlab\journal']);
        catch
            display('Cant get into correct directory!!');
        end
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
    posefromRobMaxReach=[3,8,13];
    %so after a search is done then how many did AXBAM use before doing the
    %next search, so exhastive must do the same
    aftersearch_num_poses=[3,4,3];
else
    av_time=inf;
    display(['Nothing given so : Average time per decision= ',num2str(av_time)])
    display('Need to look at the poses and find out where the scans were taken');
    error('define posefromRobMaxReach')
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

    
for i=1:size(Xsearchdata,2)
    global workspace Q r data
    display(['At search position',num2str(i), 'Currently know (obs and free) =',...
        num2str(size(Xsearchdata(i).workspace.indexedobsticles,1)+size(Xsearchdata(i).workspace.knowncoords,1))]);            
    %This makes the size of the search, cal with x^3*7*2
    numNBVanglesteps=[6,7,8];
    for searchsizeIndex=1:length(numNBVanglesteps)
        if i==1 && (searchsizeIndex==1 || searchsizeIndex==2) ;display('not doing for i==1 and first and second search');continue; end
        
        %find out where the arm is
        try    use_real_robot_GETJs(); 
        catch display('couldnt get joints from robot- updating the Q in memory regardless');
        end   
        %move to the correct place and load workspace
        workspace=Xsearchdata(i).workspace;
%         keyboard
%             load test6Exhastive9seg_workspace.mat
        
        %Set the supposed Q for here
        newQ=robot_maxreach.path(posefromRobMaxReach(i)).all_steps(end,:);
%         newQ=data.bvs(2).bestviews(3).Q;
        
        valid=0;invalidcount=0;
        while valid==0
            [valid,all_steps]=pathplanner(newQ,false,true,true,150);            
            if valid==-1
                display('Im being told that this is a dangerous position when I know this is not the case-you have control, please use exGUI to move manually to the required newQ and remove selefscanning points al the way along')            
                break;
            elseif valid==0                
                invalidcount=invalidcount+1;
                if invalidcount>10
                    display('REALLY! No Path found - you have control - USE gui to move to correct newQ ');
                    break;
                else
                    display('No Path found to get to a possible position - trying again');
                end
            end
        end
        
        %make sure we have a path 
        if invalidcount>=10 && valid~=1
            display(['Please use GUI to move to desired place which is ',num2str(newQ*180/pi),' degrees']);
            keyboard
        else
            try use_real_robot_MOVE(all_steps)
                use_real_robot_GETJs(); 
            catch display('couldnt move robot - updating the Q in memory regardless');
                %update the actual possion in memory in case it hasn't been done
                Q=newQ;
            end   
        end
        
        
        %do appropriate search
        display(['Doing exhastive search with J1-J3 steps =',num2str(numNBVanglesteps(searchsizeIndex)),' where this is x total will be (x+1)^3*7*2']);
%         if i==1 && searchsizeIndex==2;
%             %do nothing
%             display('doing nothing here')
%         else
            [data(i).totaltime(searchsizeIndex),data(i).searchspace(searchsizeIndex)]= near_exhastive_NBV_search(numNBVanglesteps(searchsizeIndex));
%         end
        
        global bestviews
        currentBview=1;
        %PATH: try and get to the position told
        while ~isfield(bestviews,'valid') || bestviews(currentBview).valid==false
            [bestviews(currentBview).valid,bestviews(currentBview).all_steps]=pathplanner(bestviews(currentBview).Q,false,true,true,30);            
            if ~bestviews(currentBview).valid                
                display(['Couldnt find a path for best view num: ',num2str(currentBview),' Removing self scannig points ever increasingly']);                
                workspace.indexedobsticles=remove_self_scanning(workspace.indexedobsticles,Q,2.5*(1+currentBview/1000));
                currentBview=currentBview+1;
            end
        end 

        display(['Found a path to best view num: ',num2str(currentBview)]);
        %remake the best views var so the top on is the best
        bestviews=bestviews(currentBview:end);
        

        %EXPLORE
        
        for cur_scan=1:aftersearch_num_poses(i) %If you want to start somewhere different eg...cur_scan=3
            nopathcount=0;
            if cur_scan>1
                while (isempty(bestviews(1+nopathcount).valid) || ...
                       bestviews(1+nopathcount).valid~=true)          
                    [bestviews(1+nopathcount).valid,bestviews(1+nopathcount).all_steps]=...
                     pathplanner(bestviews(1+nopathcount).Q,false,true,true,30);            
                 
                    if ~bestviews(1+nopathcount).valid
                        display(['At cur_scan = ',num2str(cur_scan),', Couldnt find a path for best view num: ',num2str(1+nopathcount),' Removing self scannig points ever increasingly']);                
                        workspace.indexedobsticles=remove_self_scanning(workspace.indexedobsticles,Q,2.5*(1+nopathcount/1000));
                        nopathcount=nopathcount+1;
                    end
                end 
            end

            %the cur best view is always the top best views, view
            data(i).bvs(searchsizeIndex).bestviews(cur_scan)=bestviews(1+nopathcount);
            newQ=[data(i).bvs(searchsizeIndex).bestviews(cur_scan).Q(1:5),0];
            all_steps=data(i).bvs(searchsizeIndex).bestviews(cur_scan).all_steps;
            
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
            
%             reoder best views from 2nd to last
            for cur_view=2:size(bestviews,2); temp_bestviews(cur_view-1)=bestviews(cur_view);end
            bestviews=temp_bestviews;
            order_bestviews(true); %and redo NBV volumes
            
            %print out the results for this step of this scan at this
            %Xposition
            display(['Xsearch pos',num2str(i), '.  J1-J3 steps =',num2str(numNBVanglesteps(searchsizeIndex)),...
                ' search space= ', num2str(numNBVanglesteps(searchsizeIndex)^3*7*2),...
                ', After bestview ', num2str(cur_scan), ', Currently know (obs and free) =',...
            num2str(size(workspace.indexedobsticles,1)+size(workspace.knowncoords,1))]);            

        end        
    end    
end

display('Stopping here for results to be analysed'); 
%save results to disk for later analysis
save(['doKPItest',num2str(testnum),'-',num2str(date),'_Rand',num2str(rand),'.mat'],'data');

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

%The BRUTE FORCE COMPARISONS
try load(['test',num2str(testnum),'state_data.mat']);catch; keyboard;end

plot(state_data.size_indexedobsticles+state_data.size_known)
original=state_data.size_indexedobsticles+state_data.size_known;

BF1=[original(1:4),data(1).size_allknown(1).val,data(2).size_allknown(1).val,data(3).size_allknown(1).val];
interval=interp1([1,2,3,4,5,6,7,8,9,10,12,13,14],BF1,11);
BF1=[BF1(1:11),interval,BF1(12:13)];
hold on;plot(BF1)
hold on;plot(BF1,'g')
BF2=[original(1:4),data(1).size_allknown(2).val(1:3),data(2).size_allknown(2).val(1:4),data(3).size_allknown(2).val(1:3)];
hold on;plot(BF2,'r')
BF3=[original(1:4),data(1).size_allknown(3).val(1:3),data(2).size_allknown(3).val(1:3),data(3).size_allknown(2).val(1:3)];
interval=interp1([1,2,3,4,5,6,7,8,9,10,12,13,14],BF3,11);
BF3=[BF3(1:11),interval,BF3(12:13)];
hold on;plot(BF3,'black')

% 
ORIGinfogain=original(7)+(original(11)-original(7))+original(end)-original(11)
%This is how much info was "actually" gained at correct spots for test 6
BF1infogain=BF1(7)+(BF1(11)-original(7))+BF1(end)-original(11)
BF2infogain=BF2(7)+(BF2(11)-original(7))+BF2(end)-original(11)
BF3infogain=BF3(7)+(BF3(11)-original(7))+BF3(end)-original(11)
BF1infogain/max([BF1infogain,BF2infogain,BF3infogain])
BF2infogain/max([BF1infogain,BF2infogain,BF3infogain])
BF3infogain/max([BF1infogain,BF2infogain,BF3infogain])

%from text file (interpolating since the majority of time is for the nbv
%cals and this is what was done 3,4,3 times for each one of these
BF1time=[0,0,193,193,193,215.984,215.984,215.984,215.984,208.453,208.453,208.453];
sum(BF1time)/length(BF1time)
BF2time=[0,0,324.3,324.3,324.3, 345.203,345.203,345.203,345.203,327.516,327.516,327.516];
sum(BF2time)/length(BF2time)
BF3time=[0,0,507.296,507.296,507.296,501.343,501.343,501.343,501.343,466.125,466.125,466.125];
sum(BF3time)/length(BF3time)








end

%% %%%%%
%|/     ~|
%|\PI   /|\
%%%%%%% ^^
%\subsubsection{KPI 2 - Look Around}
if tests2do(2)==1
display('KPI2 is a visual inspection, just look at it!');
load(['test',num2str(testnum),'hMesh.mat']);
plot3(hMeshdata.v(:,1),hMeshdata.v(:,2),hMeshdata.v(:,3),'r','marker','.','linestyle','none','markersize',0.1);axis equal; grid on
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
%         display('Using the 11th mesh for test6 since same env, but improved map by steph');
%         load(['test',num2str(11),'hMesh.mat']);
%     else
        load(['test',num2str(testnum),'hMesh.mat']);
%     end

    
    figure
    bridgeSection;
    hold on;
    for i=1:10;
        [u(i),sig(i),D(i).vals]=comparemaps(i,testnum,false);
    end;
  
    %fix up the pic figure
    figure(1)
    global r Q
    Q=[pi/6,pi/3,-pi/2,0,pi/2,0]
    plotdenso(r,Q);
    %set view to be desired one
    az =   71.5000;
    el =    10;
    view(az,el)
    
    camlight
    lighting gouraud
    axis equal;
    axis off;
%     uiwait(msgbox('Move figure to the way you want then do save as and patches.eps - then press ok'));
%     figure(1)
    scrsz = get(0,'ScreenSize');set(gcf,'position',[scrsz(1) scrsz(2) scrsz(3) scrsz(4)]);
    saveas(gcf,['test',num2str(testnum),'patches.png']);

%     close all;
    
    figure;       
    bar(u);
    title('Mean over 10 planes')
    u*1000
    figure; 
    bar(sig)
    title('Standard deviation over 10 planes')
    sig*1000
    
    figure
    names=[];vals=[];
    for i=1:size(D,2)
        names=[names;i*ones([size(D(i).vals,1),1])];
        %put all together and in mms (from meters originally)
        vals=[vals;D(i).vals(:)*1000];
    end
    boxplot(vals,names)
    title('Map Surface Variation Boxplots','Fontsize',30)
    ylabel('Distance To Plane (mm)','Fontsize',24)
    xlabel('Surface Number','Fontsize',24)
    set(gca,'Fontsize',20)
    scrsz = get(0,'ScreenSize');set(gcf,'position',[scrsz(1)/2 scrsz(2)/2 scrsz(3)/2 scrsz(4)/2]);
    grid on;
%     uiwait(msgbox('Fig plot up with what you want then press OK'));
    saveas(gcf,['test',num2str(testnum),'boxplot.png'])
    
    figure        
    index=GetImpLevInfo(hMeshdata.v);
    index2=find(hMeshdata.v(index,1)>-0.88);
    plot3(hMeshdata.v(index(index2),1),hMeshdata.v(index(index2),2),hMeshdata.v(index(index2),3),'r','marker','.','linestyle','none','markersize',0.1);axis equal; grid on
    
    
    figure(1)
    



end


%% %%%%%
%|/     |~
%|\PI   |~|
%%%%%%%  ~
%\subsubsection{KPI 6 - C space opened up}
if tests2do(6)==1
    
    load(['test',num2str(testnum),'hMesh.mat']);        

    load(['test',num2str(testnum),'workspace.mat']);
    
    r=rob_object;
    %average the mesh out
    global workspace r
	points_found=unique(round(hMeshdata.v/workspace.inc_size)*workspace.inc_size,'rows');
    points_found=points_found(GetImpLevInfo(points_found),:);
    clear hMeshdata
    
    qlimit=r.qlim;

%% optimal case, derive points from ply file map
    %set empty var for points
            points=[];
%             brSecfilename='bridgeSection';
if testnum==6
            brSecfilename='CASESTUDY_bridgeSection';
elseif testnum==9
            brSecfilename='AT1_bridgeSection';
elseif testnum==11
            brSecfilename='AT2_bridgeSection';
else
    error('dont know which map to use');
end
            
            brSec=plyread([brSecfilename,'.ply']);
    for i=1:size(brSec.face.vertex_indices,1)

        %have to move it up one
        verts=brSec.face.vertex_indices{i}+1;
        pnt1=[brSec.vertex.x(verts(1)),brSec.vertex.y(verts(1)),brSec.vertex.z(verts(1))];
        pnt2=[brSec.vertex.x(verts(2)),brSec.vertex.y(verts(2)),brSec.vertex.z(verts(2))];
        pnt3=[brSec.vertex.x(verts(3)),brSec.vertex.y(verts(3)),brSec.vertex.z(verts(3))];
        pnt4=[brSec.vertex.x(verts(4)),brSec.vertex.y(verts(4)),brSec.vertex.z(verts(4))];    
        points=[points;pnt1;pnt2;pnt3;pnt4];

%       inc_size=floor((pnt1-pnt2)/(workspace.inc_size/4));
%       if i>=size(brSec.face.vertex_indices,1)-8
%           close all;plot3(pnt1(1),pnt1(2),pnt1(3),'r*');hold on;plot3(pnt2(1),pnt2(2),pnt2(3),'g*');plot3(pnt3(1),pnt3(2),pnt3(3),'b*');plot3(pnt4(1),pnt4(2),pnt4(3),'black*')
%           keyboard
%       end                

        %there may be times when the box is too small so just skip it               
        try
            %gets ponts on one side
            inc_size=max([ceil(abs((pnt4-pnt1)/(workspace.inc_size/4))),2]);
            if pnt4(1)==pnt1(1); 
                pointonside1=ones([inc_size,1])*pnt1(1);
            else
                pointonside1=[pnt1(1):(pnt4(1)-pnt1(1))/(inc_size-1):pnt4(1)]';
            end
            if pnt4(2)==pnt1(2)
                pointonside1=[pointonside1,ones([inc_size,1])*pnt1(2)];
            else
                pointonside1=[pointonside1,[pnt1(2):(pnt4(2)-pnt1(2))/(inc_size-1):pnt4(2)]'];
            end                                                                         
            if pnt4(3)==pnt1(3)
                pointonside1=[pointonside1,ones([inc_size,1])*pnt1(3)];
            else
                pointonside1=[pointonside1,[pnt1(3):(pnt4(3)-pnt1(3))/(inc_size-1):pnt4(3)]'];
            end

            %gets ponts on other side
            inc_size=max([ceil(abs((pnt3-pnt2)/(workspace.inc_size/4))),2]);
            if pnt2(1)==pnt3(1)
                pointonside2=ones([inc_size,1])*pnt2(1);
            else
                pointonside2=[pnt2(1):(pnt3(1)-pnt2(1))/(inc_size-1):pnt3(1)]';
            end
            if pnt2(2)==pnt3(2)
                pointonside2=[pointonside2,ones([inc_size,1])*pnt2(2)];
            else
                pointonside2=[pointonside2,[pnt2(2):(pnt3(2)-pnt2(2))/(inc_size-1):pnt3(2)]'];
            end                                                                         
            if pnt2(3)==pnt3(3)
                pointonside2=[pointonside2,ones([inc_size,1])*pnt2(3)];
            else
                pointonside2=[pointonside2,[pnt2(3):(pnt3(3)-pnt2(3))/(inc_size-1):pnt3(3)]'];
            end

            %gets all points in between
            for j=1:length(pointonside1)

                startpnt=pointonside1(j,:);
                endpnt=pointonside2(j,:);

                inc_size=max([ceil(abs((endpnt-startpnt)/(workspace.inc_size/4))),2]);
                if endpnt(1)==startpnt(1)
                    pointonline=ones([inc_size,1])*startpnt(1);
                else
                    pointonline=[startpnt(1):(endpnt(1)-startpnt(1))/(inc_size-1):endpnt(1)]';
                end
                if endpnt(2)==startpnt(2)
                    pointonline=[pointonline,ones([inc_size,1])*startpnt(2)];
                else
                    pointonline=[pointonline,[startpnt(2):(endpnt(2)-startpnt(2))/(inc_size-1):endpnt(2)]'];
                end                                                                         
                if endpnt(3)==startpnt(3)
                    pointonline=[pointonline,ones([inc_size,1])*startpnt(3)];
                else
                    pointonline=[pointonline,[startpnt(3):(endpnt(3)-startpnt(3))/(inc_size-1):endpnt(3)]'];
                end

%                 if i>=size(brSec.face.vertex_indices,1)-8
%                     hold on;plot3(pointonline(:,1),pointonline(:,2),pointonline(:,3),'b.')
%                     keyboard
%                 end                

                points=[points;pointonline];
            end
        end
        
        points=unique(round(points/(workspace.inc_size))*(workspace.inc_size),'rows');
    
    end


%%%%%%%%%%%%%%%%%%%%
%% Compare 2 point sets
    points=            points(GetImpLevInfo(unique(round(      points/(workspace.inc_size))*(workspace.inc_size),'rows')),:);
    points_found=points_found(GetImpLevInfo(unique(round(points_found/(workspace.inc_size))*(workspace.inc_size),'rows')),:);
    %remove points found below the lowest ideal case theshold since that is
    %what we have compared the maps on
    points_found=points_found(find(points_found(:,3)>min(points(:,3))),:);
    
    %compare the 11th patch (the whole map)
    [u,sig]=comparemaps(11,testnum,true,brSecfilename);
    display(['Mean is ', num2str(u),' and Standard deviation is',num2str(sig)]) ;
    
    incval=7; %degrees between steps
    numvals=floor((qlimit(:,2)-qlimit(:,1))/(incval*pi/180));

    %if do 100 check a second then this will take 
    display(['if do 500 checks a second then search space of ',num2str(numvals(1)*numvals(2)*numvals(3)) ,' will take ',num2str(((numvals(1)*numvals(2)*numvals(3))/500)/60),'minutes per test']);

    %Test
    for testcase=1:2
        validcount(testcase)=0;
        failedcount(testcase)=0;
        
        %1 is ideal, 2 is real case
        if testcase==1
            obstaclepoints=points;        
        elseif testcase==2
            obstaclepoints=points_found;        
        end
        
%         display('stopping here before doing extensive search');keyboard;

        %doing the valid challenge
        tic
        
        for i=qlimit(1,1):(incval*pi/180):qlimit(1,2)
            for j=qlimit(2,1):(incval*pi/180):qlimit(2,2)
                for k=qlimit(3,1):(incval*pi/180):qlimit(3,2)
                    if check_path_for_col([i,j,k,0,0,0],obstaclepoints)
                        validcount(testcase)=validcount(testcase)+1;
                    else
                        failedcount(testcase)=failedcount(testcase)+1;
                    end
                end
            end
        end
        toc
    end

    display(['Ideal Case: Valid=',num2str(validcount(1)),' Failed=',num2str(failedcount(1)),' which is ',num2str(validcount(1)/(failedcount(1)+validcount(1))*100),'%']);
    display(['Actual Case: Valid=',num2str(validcount(2)),' Failed=',num2str(failedcount(2)),' which is ',num2str(validcount(2)/(failedcount(2)+validcount(2))*100),'%']);
    if validcount(1)>validcount(2)
        display(['Too many fails: Comparitively the ratio of what we got to what we should of got is ', num2str(validcount(2)/validcount(1))]);
    else
        display(['Not enough fails: Comparitively the ratio of what we got to what we should of got is ', num2str(validcount(1)/validcount(2))]);
    end
        
%     keyboard

    
end


%% Overall catch statement
catch
    lasterr
    tempE=lasterror;
    display('question tempE stack for error details if above was not good enough.... you have control');
    keyboard
end
