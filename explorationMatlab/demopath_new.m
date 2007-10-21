%% demopath
%
% *Description:*  This function is sent the steps of a path and then this
% function will go through each step and show the path. There is also the
% option not to send in steps and it will load path_data.mat and then
% gather data about the movement of joints around the current step. This
% can be used to produce an image of the configuration space


%% Function Call
% 
% *Inputs:* 
%
% _all_steps_ (6*m double) these are the joint configs radians
%
% *Returns:* NULL

function demopath(all_steps)

%% Variables
global r Q densoobj workspace optimise

if size(r,1)<1
    error('robot has not been setup')
end

%setup path data and obsticle data
if nargin==0
    %make a test case and then act these out
    %testpathplanner(30);
    load pathdata.mat;
    DISPON=false;
    %to check the differentials
    checkstepsize=true; 
      overalltotalmovement=0;
      overallabsstart_end_diff=0;
      
    diffrad2check=optimise.max_angle_for123;
    showOBpoints=false;;
    
    points=workspace.indexedobsticles;
    check_all_Js=false;
    jointlimits=r.qlim;
    allFFresults.joint1_data=[];
    allFFresults.joint2_data=[];
    allFFresults.joint3_data=[];
    allFFresults.joint4_data=[];
    allFFresults.joint5_data=[];
    allFFresults.joint6_data=[];
    angle_res=0.01; %radians
    n = r.n;
    L = r.link;
else %we are just demoing the path that has been passed
    pathdata(1).all_steps=all_steps;
    DISPON=true;
    points=workspace.indexedobsticles;
    check_all_Js=false;
    checkstepsize=false;
    showOBpoints=false;
end

%% Setup the figure (if required)
if DISPON
    %fig=figure(2); 
    if size(points,1)>0 && showOBpoints
        plot3(points(:,1),points(:,2),points(:,3),'marker','.','Color',[.2,.2,.1],'linestyle','none'); 
    end        
    view(3);axis([-1 1 -1 1 0 1.5]);
%     colordef white;
%     grid on;
    hold on;
end


%% Go through each path
for current_path=1:size(pathdata,2)
    if current_path>2 && DISPON; display(strcat('Now working on path num:',num2str(current_path)));end;
    if checkstepsize==true
        all_stepsdiff=diff(pathdata(current_path).all_steps);
        if ~isempty(find(all_stepsdiff(:,1)>diffrad2check(1)|...
                         all_stepsdiff(:,2)>diffrad2check(2)|...
                         all_stepsdiff(:,3)>diffrad2check(3),1))
            display('There is a problem with this path - exceeding min J1-J3 step limits');
        end
        if ~isempty(find(all_stepsdiff(:,4)>20*pi/180,1)) && DISPON
            display('joints 4has moved more than 20 deg in a step');
        end
        if ~isempty(find(all_stepsdiff(:,5)>30*pi/180,1)) && DISPON
            display('joints 5 has moved more than 30 deg in a step');
        end
        if ~isempty(find(all_stepsdiff(:,4)>30*pi/180,1)) && DISPON
            display('joints 6 has moved more than 30 deg in a step');
        end
        %work out the total actual movement for the path as compared to
        %direct path
        totalmovement=[sum(abs(all_stepsdiff(:,1))),sum(abs(all_stepsdiff(:,2))),sum(abs(all_stepsdiff(:,3)))];
        absstart_end_diff=abs(pathdata(current_path).all_steps(1,1:3)-pathdata(current_path).all_steps(end,1:3));
        diffaverage(current_path,:)=totalmovement./absstart_end_diff;
        overalltotalmovement=overalltotalmovement+totalmovement;
        overallabsstart_end_diff=overallabsstart_end_diff+absstart_end_diff;
    end


    % Show ellispes if required
    if rand>1 show_ellipses=true;
    else show_ellipses=false;        
    end
    
    % Determine the start and end point
    start_t=fkine(r,pathdata(current_path).all_steps(1,:));
    end_t=fkine(r,pathdata(current_path).all_steps(end,:));
    pointsH=plot3([start_t(1,4) end_t(1,4)],[start_t(2,4) end_t(2,4)],[start_t(3,4) end_t(3,4)],'b');
    
    %go through and actually animate each step of this particular path
    for current_step=1:size(pathdata(current_path).all_steps,1)        
        nextQ=pathdata(current_path).all_steps(current_step,:);        
        if DISPON
            plotdenso(r, nextQ, show_ellipses, show_ellipses);        
        
            %work out where the end effector is
            t=fkine(r,nextQ);
            if current_step==1; prev_t=start_t; end;

            temppointsH=plot3([prev_t(1,4) t(1,4)],[prev_t(2,4) t(2,4)],[prev_t(3,4) t(3,4)],'Color',[1 0 0]);
            prev_t=t;
            %add plot handle to the stack so that it can be deleted
            pointsH=[pointsH;temppointsH];        

            drawnow;        
        end
        
        %it will go through an check either side (completely) of the
        %current joint to see where collisions could have occured
        if check_all_Js
            for current_joint=1:3
                tempresult=[];
                tempnextQ=nextQ;
                for current_angle=jointlimits(current_joint,1):angle_res:jointlimits(current_joint,2)
                    tempnextQ(current_joint)=current_angle;

                    t = r.base;
                    FF_OK=1;%force field ok
                    for piece=1:n
                        t = t * L{piece}(tempnextQ(piece));    
                        if ~check_FF(t,densoobj(piece+1).ellipse,points)
                            FF_OK=0;
                            break
                        end
                    end
                    tempresult=[tempresult;FF_OK];                   
                end

                %change the value where the current angleis
                try tempresult(round((-jointlimits(current_joint,1)+nextQ(current_joint))/angle_res))=0.5;
                catch if round((-jointlimits(current_joint,1)+nextQ(current_joint))/angle_res)<=0
                        tempresult(1)=0.5;
                    else
                        tempresult(end)=0.5;
                    end
                end
                
                %add the data about all the joints (if changes) at the
                %current step to the overall data
                if current_joint==1
                    allFFresults.joint1_data=[allFFresults.joint1_data,tempresult];
                elseif current_joint==2 
                    allFFresults.joint2_data=[allFFresults.joint2_data,tempresult];
                elseif current_joint==3
                    allFFresults.joint3_data=[allFFresults.joint3_data,tempresult];                
                elseif current_joint==4
                    allFFresults.joint4_data=[allFFresults.joint4_data,tempresult];                
                elseif current_joint==5
                    allFFresults.joint5_data=[allFFresults.joint5_data,tempresult];                
                elseif current_joint==6
                    allFFresults.joint6_data=[allFFresults.joint6_data,tempresult];                
                end      
            end
        end
    end    
    
    if DISPON
        axis([-1 1 -1 1 0 1.5]);
%         grid on;
%         rotate3d on;
        %pause(3)
%         rotate3d off;
        %print('-dpng', sprintf('%s_%03.0f_%03.0f.png', 'demopath',current_path, current_step));
        
    end
    
    %try and delete the intermediate points of movement
    for current_h=1:size(pointsH,1)
        try delete(pointsH(current_h)); end;
    end
end

%if we want to show results of the checks on the stepsize
if checkstepsize==true
    display(['Ave(permove) Act Q move/hypothetical dir j move:[J1->3]=[',...
        num2str(sum(diffaverage(:,1))/size(diffaverage(:,1),1)),',',...
        num2str(sum(diffaverage(:,2))/size(diffaverage(:,2),1)),',',...
        num2str(sum(diffaverage(:,3))/size(diffaverage(:,3),1)),']']);
    display(['Total actMove/endjsdiff (ratio): [J1->3]=[',...
        num2str(overalltotalmovement./overallabsstart_end_diff)]);
end

%% If we wish to save the results
if check_all_Js
    %keyboard
    save('allFFresults.mat','allFFresults');
end