%% pathplanner
%
% *Description:*  this function goes from where we are at Q to where we want
% to be at newQ and checks for collisions

%% Function Call
% 
% * *Inputs:* 
%   newQ (1*6 double) radians - The new joint configuration that want to be in
%   tryalternate (binary) If you want to try and get another possible joint configuration
%   check_arm_perms (binary)  If you want to check each of the joints 1-3 perms for a solution
%   useMiddleQ2 (binary) If you want to try and put a second point in from middleQ to end if there is no direct path
%   numofPPiterations (int) This is how many times you want to go through the middleQ paths
%   disON (binary) If you want to see the print outs of the working
% * *Returns:* 
%   pathfound =0 (no path), -1 (not valid newQ), 1 (path found)
%   all_steps = 6 collums of joints * many steps

function [pathfound,all_steps] = pathplanner(newQ,tryalternate,check_arm_perms,useMiddleQ2,numofPPiterations,disON)

%% Variables
global r Q optimise workspace
  
%intiallise all the steps through the path to empty
all_steps=[];

%% check if we are at the end (exit if we are)
if isempty(find(newQ~=Q, 1))
    pathfound=1;
    return;
end

%% Check passed arguments set defaults where appropriate
if nargin<6
    disON=false;
    if nargin<5
        %default nu of interations assuming that there IS an alternate solution
        numofPPiterations=optimise.numofPPiterations;
        if nargin<4
            % It will make more than one middle possition and try and reach this
            useMiddleQ2=true;
            if nargin<3        
                %if we want to use the different arm movement permutations
                check_arm_perms=true;
                if nargin<2
                    %default is to get an alternate pose just in case
                    tryalternate=true;
                    if nargin==0; error('You must pass at least a newQ value to try and reach from the current (global) Q');end
                end
            end
        end
    end
end

%these are the obstacle point within the arm range that are present during this iteration of path planning
obsticle_points=workspace.indexedobsticles(GetImpLevInfo(workspace.indexedobsticles),:);

%if we are not going to try and use alternate solution then go through 1/2 amount of times
if ~tryalternate
    numofPPiterations=numofPPiterations/2;
end

%what the max angle is or the joints 1,2,3 so we work out steps from this
max_angle_for123=optimise.max_angle_for123;

%this is parameters of the robot
qlimits=r.qlim;

%% check that the initial desired end joint config is safe and possible
if ~check_path_for_col(newQ,obsticle_points); 
    pathfound=-1; 
    return; 
end
% Check joint limits
if ~isempty(find(newQ'<qlimits(:,1), 1)) || ~isempty(find(newQ'>qlimits(:,2), 1))
    pathfound=-1;
    return;
end
   
%% Check if there is a direct path
[pathfound,all_steps]=checkdirectpath(Q,newQ,max_angle_for123,check_arm_perms,obsticle_points);
%since a direct path is possible and safe return with this path
if pathfound; 
    if disON; display('Complete safe path found-direct');end; 
    return; 
end

%% If we are trying to find an alternate end work this out
if tryalternate
    try 
        requiredT=fkine(r,newQ);
        if disON; display('using new pose selection method which includes collision detection');end
        [alternate_newQ,valid_pose]=streamOnto_mine_manystarts(r,requiredT(1:3,4),requiredT(1:3,3),Q);
        if ~valid_pose 
            numofPPiterations=numofPPiterations/2;
            tryalternate=0; %we wont try this section again
            if disON; display('Did not find a valid solution using many starts');end        
        end            
    catch
        numofPPiterations=numofPPiterations/2;
        tryalternate=0; %we wont try this section again
    end
end
%try and get a path with the alternate end and same begining
if tryalternate
    [pathfound,all_steps]=checkdirectpath(Q,alternate_newQ,max_angle_for123,check_arm_perms,obsticle_points);
    if pathfound; 
        if disON; display('Complete safe path found');end; 
        return; 
    end
end


%% Go through required number of iterations and try and find a path
for gothroughtimes=1:numofPPiterations    
if pathfound==0
    if disON; 
        display('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
        display(strcat('Currently going through for the:',num2str(gothroughtimes)));
        display('-----------------------------------------------')
    end

    %setup a start Q variable that won't alter the global
    startQ=Q;
       
    %this will try the Alternat path on the even numbers for gothroughtimes
    if (-1)^(gothroughtimes)==1 && tryalternate
        endQ=alternate_newQ;
        if disON; display(strcat('Using an alternate end Q:',num2str(alternate_newQ)));end
    else
        % just do the normal endQ which is newQ
        endQ=newQ;
    end    

    %this var will hold all the intermediate steps between Q and newQ
    all_steps=startQ;  
    
    %randomly select a middleQ with first 3 joints randomly chosen between bounds
    middleQ=pick_a_middleQ(startQ,endQ);

    if disON; display(strcat('Splitting path into two parts with middle as:',num2str(middleQ)));end
    
    [pathfound,all_steps]=checkdirectpath(startQ,middleQ,max_angle_for123,check_arm_perms,obsticle_points);    
    %if no path found found go to the next iteration
    if pathfound==0; 
        %else it is a complete failure
        if disON; display('FAILED: path finding failure: couldnt get start->middle');end; 
        continue; 
    else
        if disON; display('With the path now split, we have got halfway, now trying to get whole way');end
        [pathfound,additional_steps]=checkdirectpath(middleQ,endQ,max_angle_for123,check_arm_perms,obsticle_points);
        if pathfound==0; 
            if disON; display('path finding failure: couldnt get middle->end');end
            if useMiddleQ2
                %randomly select another middleQ2 between middleQ and endQ
                middleQ2=pick_a_middleQ(middleQ,endQ);
                if disON; display(strcat('Splitting path into 3 parts with middle2 (middle of middle->end) as:',num2str(middleQ2)));end
                [pathfound_1,additional_steps1]=checkdirectpath(middleQ,middleQ2,max_angle_for123,check_arm_perms,obsticle_points); 
                [pathfound_2,additional_steps2]=checkdirectpath(middleQ2,endQ,max_angle_for123,check_arm_perms,obsticle_points);            
                if pathfound_1 && pathfound_2
                    all_steps=[all_steps;additional_steps1;additional_steps2];
                    if disON; display('Complete safe path found_with 2 mid points');end; 
                    pathfound=1;
                    return;
                else               
                    %else go on to next path
                    if disON; display('FAILED: path finding failure: couldnt get middle->middle2 or middle2->end');end;
                    continue; %to the next itteration
                end
            else %we don't want to try and get another point, simple try another
                continue;
            end
        else; all_steps=[all_steps;additional_steps];
            if disON; display('Complete safe path found_with 1 mid point');end; 
            return;
        end
    end;     

else % asolution has been found so you can break out
    break;
end %end the IF path found
end; %end overall for loop for num of iterations

   
%% FUNCTION: check between a start and end point to see if there is a path
function [pathfound,all_steps]=checkdirectpath(startQ,endQ,max_angle_for123,check_arm_perms,obsticle_points)
%even if it is half a whole path still start with the startQ
all_steps=startQ;

%how many increments to go through to get to end
num_inc=ceil(max([abs(endQ(1)-startQ(1))/max_angle_for123(1),...
                  abs(endQ(2)-startQ(2))/max_angle_for123(2),...
                  abs(endQ(3)-startQ(3))/max_angle_for123(3)]));
%since we may not be at our destination but only joint 4,5,6 may change and
%so we will have num_inc==0 so we have to make it go through in atleast 1 step
num_inc=max(num_inc,1);

%this is the size of the increment values
inc_val=(endQ-startQ)/num_inc;

%make the path up out of the smaller steps
for i=1:num_inc
    all_steps=[all_steps;startQ+i*inc_val];
end
%if the final step is not the actual end then just add it on
if ~isempty(find(all_steps(end,:)-endQ~=0)>0)
    all_steps=[all_steps;endQ];
end
pathfound=check_path_for_col(all_steps,obsticle_points);
if pathfound==1
    return
%if we are to go through and check each arm permutation as well for a collsion free path 
elseif check_arm_perms
    %calculate new increment value because each angle is separate there is
    %no need to make so many steps
    %how many increments to go through to get to end
    num_inc=[abs(endQ(1)-startQ(1))/max_angle_for123(1),...
            abs(endQ(2)-startQ(2))/max_angle_for123(2),...
            abs(endQ(3)-startQ(3))/max_angle_for123(3)];
    num_inc=[ceil(num_inc),ones([1,3])*ceil(sum(num_inc(1:3)))];
    if num_inc==0
        keyboard
    end

    %this is the size of the increment values
    inc_val=(endQ-startQ)./num_inc;
    jointperm=perms(1:3);
    %make the path up out of the smaller steps for each of joint individually
    %Check each per for a path and return if there is one
    for currentperm=1:size(jointperm,1)
        all_steps=startQ;
        for jointnum=jointperm(currentperm,:)                      
            for current_inc=1:num_inc(jointnum)
                if     jointnum==1; all_steps=[all_steps;all_steps(end,:)+[inc_val(1),0,0,inc_val(4:6)]];
                elseif jointnum==2; all_steps=[all_steps;all_steps(end,:)+[0,inc_val(2),0,inc_val(4:6)]];
                elseif jointnum==3; all_steps=[all_steps;all_steps(end,:)+[0,0,inc_val(3),inc_val(4:6)]];end                
            end
        end
        if ~isempty(find(all_steps(end,:)-endQ~=0)>0);all_steps=[all_steps;endQ];end
        pathfound=check_path_for_col(all_steps,obsticle_points);
        if pathfound==1; return; end
    end        
end

%% FUNCTION: select middleQ with 1st 3 qs rand chosen within bounds 
% then the joint4-6 are simply between where they are moving. Keep on going
% through until to find a valid middle point
function middleQ=pick_a_middleQ(startQ,endQ)
global r workspace densoobj

n = r.n;
L = r.link;
qlimits=r.qlim;

result_row=[0,0,0,0,0,0];
while ~isempty(find(result_row==0, 1))
    %this gets the value for joints 2 and 3 so they are on
    %oppisite sides of their centers
    randnum=rand();
    if randnum>0.5
        randnum=(randnum)^2;
        joint2=randnum;
        joint3=1-randnum;
    else %it is on the other side
        randnum=sqrt(randnum);
        joint2=randnum;
        joint3=1-randnum;
    end
    middleQ=[rand()*(-qlimits(1,1)+qlimits(1,2))+qlimits(1,1),...
             joint2*(-qlimits(2,1)+qlimits(2,2))+qlimits(2,1),...
             joint3*(-qlimits(3,1)+qlimits(3,2))+qlimits(3,1),...
             startQ(4:6) + (endQ(4:6)-startQ(4:6))/2];
    t = r.base;
    result_row=[];
    %make sure the randomly gained solution is not impossible to reach
    for piece=1:n
        t = t * L{piece}(middleQ(piece));
        tempresult=check_FF(t,densoobj(piece+1).ellipse,workspace.indexedobsticles);                
        result_row=[result_row,tempresult];
        if tempresult==0
            break; %break out of for loop since this one is not valid
        end
    end
end %end while (trying to split path) loop
