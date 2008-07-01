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
global r Q densoobj workspace

if size(r,1)<1
    error('robot has not been setup')
end

%setup path data and obsticle data
if nargin==0
    %make a test case and then act these out
    %testpathplanner(30);
    load pathdata.mat;
    DISPON=false;
    points=workspace.indexedobsticles;
    check_all_Js=true;
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
end

%% Setup the figure (if required)
if DISPON
    %fig=figure(2); 
%     close all;
    if size(points,1)>0 
        plot3(points(:,1),points(:,2),points(:,3),'marker','.','Color',[.2,.2,.1],'linestyle','none'); 
    end        
    view(3);axis([-1 1 -1 1 0 1.5]);colordef white;grid on;hold on;
%     camlight
end


%% Go through each path
for current_path=1:size(pathdata,2)
    if current_path>2; display(strcat('Now working on path num:',num2str(current_path)));end;

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
            pause(0.3);
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
        axis([-1 1 -1 1 0 1.5]);grid on;rotate3d on;
        %pause(3)
        rotate3d off;
        %print('-dpng', sprintf('%s_%03.0f_%03.0f.png', 'demopath',current_path, current_step));
        
    end
    
    %try and delete the intermediate points of movement
    for current_h=1:size(pointsH,1)
        try delete(pointsH(current_h)); end;
    end
end

%% If we wish to save the results
if check_all_Js
    %keyboard
    save('allFFresults.mat','allFFresults');
end
