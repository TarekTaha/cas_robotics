%% pathplanner_new
%
% *Description:*  NEW version of function which goes from where we are at Q
% to where we want to be at newQ and checks for collisions

%% Function Call
% 
% *Inputs:* 
%
% _newQ_ (1*6 double) radians - The new joint configuration that want to be in
%
% _tryalternate_ (binary) If you want to try and get another possible joint configuration
%
% _check_arm_perms_ (binary)  If you want to check each of the joints 1-3 perms for a solution
%
% _useMiddleQ2_ (binary) If you want to try and put a second point in from middleQ to end if there is no direct path
%
% _numofPPiterations_ (int) This is how many times you want to go through the middleQ paths
%
% _disON_ (binary) If you want to see the print outs of the working
%
% _currQ_ (1*6 double) radians - Is not usually passed and is set to Q however if passed planning will be from here
%
% *Returns:* 
%
% _pathfound_ =0 (no path), -1 (not valid newQ), 1 (path found)
%
% _all_steps_ = 6 collums of joints * many steps

function [pathfound,all_steps] = pathplanner_new(newQ,tryalternate,check_arm_perms,useMiddleQ2,numofPPiterations,disON,currQ)

%% Variables
global r Q optimise workspace robot_maxreach
  
%intiallise all the steps through the path to empty
all_steps=[];


%% Check passed arguments set defaults where appropriate
if nargin<7
    currQ=Q;
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
end

if size(currQ,2)>6
%   display('You should pass in newQ as a 6*1 matrix, all steps will be returned as a 6*many')
  currQ=currQ(1:6);
  newQ=newQ(:,1:6);
  make7jointrobot=true;
else
  make7jointrobot=false;
end

%Change Q to currQ

%% check if we are at the end (exit if we are)
if isempty(find(newQ~=currQ, 1))
    pathfound=1;
    return;
end

%these are the obstacle point within the arm range that are present during this iteration of path planning
obsticle_points=workspace.indexedobsticles(GetImpLevInfo(workspace.indexedobsticles),:);

%if we are not going to try and use alternate solution then go through 1/2 amount of times
if ~tryalternate
    numofPPiterations=numofPPiterations/2;
end

%what the max angle is or the joints 1,2,3 so we work out steps from this
max_angle_for123=optimise.max_angle_for123;
maxangleJ4to6=optimise.maxangleJ4to6;
minjointres=robot_maxreach.minjointres;

%this is parameters of the robot
qlimits=r.qlim;
%make sure it is the correct size
qlimits=qlimits(1:6,:);

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
[pathfound,all_steps]=checkdirectpath(currQ,newQ,max_angle_for123,check_arm_perms,obsticle_points,maxangleJ4to6,minjointres);
%since a direct path is possible and safe return with this path
if pathfound; 
    if disON; display('Complete safe path found-direct');end; 
    %if we need to pad with zeros at end
    if make7jointrobot; all_steps=padarray(all_steps,[0,1],'post');end          
    return; 
end

%% If we are trying to find an alternate end work this out
if tryalternate
    try 
        requiredT=fkine(r,newQ);
        if disON; display('using new pose selection method which includes collision detection');end
        [alternate_newQ,valid_pose]=streamOnto_mine_manystarts(r,requiredT(1:3,4),requiredT(1:3,3),currQ);
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
    [pathfound,all_steps]=checkdirectpath(currQ,alternate_newQ,max_angle_for123,check_arm_perms,obsticle_points,maxangleJ4to6,minjointres);
    if pathfound; 
        if disON; display('Complete safe path found');end; 
        if make7jointrobot; all_steps=padarray(all_steps,[0,1],'post');end   
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
    startQ=currQ;
       
    %this will try the Alternat path on the even numbers for gothroughtimes
    if (-1)^(gothroughtimes)==1 && tryalternate
        endQ=alternate_newQ;
        if disON; display(strcat('Using an alternate end Q:',num2str(alternate_newQ)));end
    else
        % just do the normal endQ which is newQ
        endQ=newQ;
    end    

    %this var will hold all the intermediate steps between currQ and newQ
    all_steps=startQ;  
    
    %randomly select a middleQ with first 3 joints randomly chosen between bounds
    middleQ=pick_a_middleQ(startQ,endQ);

    if disON; display(strcat('Splitting path into two parts with middle as:',num2str(middleQ)));end
    
    [pathfound,all_steps]=checkdirectpath(startQ,middleQ,max_angle_for123,check_arm_perms,obsticle_points,maxangleJ4to6,minjointres);    
    %if no path found found go to the next iteration
    if pathfound==0; 
        %else it is a complete failure
        if disON; display('FAILED: path finding failure: couldnt get start->middle');end; 
        continue; 
    else
        if disON; display('With the path now split, we have got halfway, now trying to get whole way');end
        [pathfound,additional_steps]=checkdirectpath(middleQ,endQ,max_angle_for123,check_arm_perms,obsticle_points,maxangleJ4to6,minjointres);
        if pathfound==0; 
            if disON; display('path finding failure: couldnt get middle->end');end
            if useMiddleQ2
                %randomly select another middleQ2 between middleQ and endQ
                middleQ2=pick_a_middleQ(middleQ,endQ);
                if disON; display(strcat('Splitting path into 3 parts with middle2 (middle of middle->end) as:',num2str(middleQ2)));end
                [pathfound_1,additional_steps1]=checkdirectpath(middleQ,middleQ2,max_angle_for123,check_arm_perms,obsticle_points,maxangleJ4to6,minjointres); 
                [pathfound_2,additional_steps2]=checkdirectpath(middleQ2,endQ,max_angle_for123,check_arm_perms,obsticle_points,maxangleJ4to6,minjointres);            
                if pathfound_1 && pathfound_2
                    all_steps=[all_steps;additional_steps1;additional_steps2];
                    if disON; display('Complete safe path found_with 2 mid points');end; 
                    pathfound=1;
                    if make7jointrobot; all_steps=padarray(all_steps,[0,1],'post');end   
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
            if make7jointrobot; all_steps=padarray(all_steps,[0,1],'post');end   
            return;
        end
    end;     

else % asolution has been found so you can break out
    break;
end %end the IF path found
end; %end overall for loop for num of iterations

   
%% FUNCTION: check between a start and end point to see if there is a path
function [pathfound,all_steps]=checkdirectpath(startQ,endQ,max_angle_for123,check_arm_perms,obsticle_points,maxangleJ4to6,minjointres)

%even if it is half a whole path still start with the startQ
all_steps=startQ;


%how many increments to go through to get to end
num_inc=ceil(max([abs(endQ(1)-startQ(1))/max_angle_for123(1),...
                  abs(endQ(2)-startQ(2))/max_angle_for123(2),...
                  abs(endQ(3)-startQ(3))/max_angle_for123(3),...
                  abs(endQ(4)-startQ(4))/maxangleJ4to6,...
                  abs(endQ(5)-startQ(5))/maxangleJ4to6,...
                  abs(endQ(6)-startQ(6))/maxangleJ4to6]));
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
%     all_steps=path_minimise(all_steps);
    return
%if we are to go through and check each arm permutation as well for a
%collsion free path as long as there is more than 1 joint different between
%start and end
elseif check_arm_perms && size(find(startQ(1:3)-endQ(1:3)==0),2)<2;
    
    %find any values that are the same and add eps to the start value
    jointsthesame=find(startQ(1:3)-endQ(1:3)==0);
    if size(jointsthesame,2)>0
        startQ(jointsthesame)=startQ(jointsthesame)+eps;
    end

    %calculate new increment value because each angle is separate there is
    %no need to make so many steps
    %how many increments to go through to get to end
    num_inc=ceil([abs(endQ(1)-startQ(1))/max_angle_for123(1),...
            abs(endQ(2)-startQ(2))/max_angle_for123(2),...
            abs(endQ(3)-startQ(3))/max_angle_for123(3),...
            abs(endQ(4)-startQ(4))/maxangleJ4to6,...
            abs(endQ(5)-startQ(5))/maxangleJ4to6,...
            abs(endQ(6)-startQ(6))/maxangleJ4to6]);
        
        num_inc(num_inc==0)=1;
        
%      num_inc=[ceil(num_inc),ones([1,3])*ceil(sum(num_inc(1:3)))];
    

    if num_inc==0
        keyboard
    end

    %this is the size of the increment values
    inc_val=(endQ-startQ)./num_inc;

%% %Additional Combs
% combs=
%       1 & 2,3
%       2 & 3,1
%       3 & 1,2

%       1,2 & 3
%       2,3 & 1
%       3,1 & 2


jointcombs= [1,2,3;2,3,1;3,1,2];
    %make the path up out of the smaller steps for each of joint individually
    %Check each per for a path and return if there is one
    for firstcombo=1:2
        for currentcomb=1:size(jointcombs,1)
            % work out the first 3 perms
            temp1=[startQ(jointcombs(currentcomb,1)):inc_val((jointcombs(currentcomb,1))):endQ(jointcombs(currentcomb,1))]';
            temp2=[startQ(jointcombs(currentcomb,2)):inc_val((jointcombs(currentcomb,2))):endQ(jointcombs(currentcomb,2))]';
            temp3=[startQ(jointcombs(currentcomb,3)):inc_val((jointcombs(currentcomb,3))):endQ(jointcombs(currentcomb,3))]';
            %check largest size
            sizetomakeall=max([size(temp1,1),size(temp2,1),size(temp3,1)]);
            %make all mats same size by populating with end value
            temp1=[temp1;temp1(end)*ones([sizetomakeall-size(temp1,1),1])];
            temp2=[temp2;temp2(end)*ones([sizetomakeall-size(temp2,1),1])];
            temp3=[temp3;temp3(end)*ones([sizetomakeall-size(temp3,1),1])];

            %Now work out how to combine these
            % start off with joint 1 which is either to move or the start state
            if jointcombs(currentcomb,1)==1
                all_steps=[temp1;...
                           temp1(end)*ones([size(temp1,1),1])];
            elseif jointcombs(currentcomb,2)==1
                all_steps=[temp2(1)*ones([size(temp2,1),1]);...
                            temp2];
            elseif jointcombs(currentcomb,3)==1
                all_steps=[temp3(1)*ones([size(temp3,1),1]);...
                            temp3];
            end
            %for JOINT2 and JOINT3
            for cur_joint=2:3
                if jointcombs(currentcomb,1)==cur_joint 
                    all_steps=[all_steps,[temp1;...
                                         temp1(end)*ones([size(temp1,1),1])]];
                elseif firstcombo==2 && jointcombs(currentcomb,2)==cur_joint
                    all_steps=[all_steps,[temp2;...
                                         temp2(end)*ones([size(temp2,1),1])]];
                elseif firstcombo==1 && jointcombs(currentcomb,2)==cur_joint
                    all_steps=[all_steps,[temp2(1)*ones([size(temp2,1),1]);...
                                         temp2]];
                elseif jointcombs(currentcomb,3)==cur_joint
                    all_steps=[all_steps,[temp3(1)*ones([size(temp3,1),1]);...
                                         temp3]];
                end
            end
            
            %fill size of last 3 joints then tac onto all_steps
            try %Joint 4
                temp4=[startQ(4):(endQ(4)-startQ(4))/(size(all_steps,1)-1):endQ(4)]';
                if isempty(temp4);temp4=0;end
            catch
                temp4=0;
            end

            try %Joint 5
                temp5=[startQ(5):(endQ(5)-startQ(5))/(size(all_steps,1)-1):endQ(5)]';
                if isempty(temp5);temp5=0;end
            catch
                temp5=0;
            end

            try %Joint 5
                temp6=[startQ(6):(endQ(6)-startQ(6))/(size(all_steps,1)-1):endQ(6)]';
                if isempty(temp6);temp6=0;end
            catch
                temp6=0;
            end
%             try
            % resize 45 and 6 if necessary either up or down
            if size(temp4,1)>size(all_steps,1)
                display('speeding up speed of 4th joint');
                temp4(size(all_steps,1))=temp4(end);
                temp4=temp4(1:size(all_steps,1));
            elseif size(temp4,1)<size(all_steps,1)
                temp4=[temp4;temp4(end)*ones([size(all_steps,1)-size(temp4,1),1])];
            end

            if size(temp5,1)>size(all_steps,1)
                display('speeding up speed of 5th joint');
                temp5(size(all_steps,1))=temp5(end);
                temp5=temp5(1:size(all_steps,1));
            elseif size(temp5,1)<size(all_steps,1)
                temp5=[temp5;temp5(end)*ones([size(all_steps,1)-size(temp5,1),1])];
            end

            if size(temp6,1)>size(all_steps,1)
                display('speeding up speed of 6th joint');
                temp6(size(all_steps,1))=temp6(end);
                temp6=temp6(1:size(all_steps,1));
            elseif size(temp6,1)<size(all_steps,1)
                temp6=[temp6;temp6(end)*ones([size(all_steps,1)-size(temp6,1),1])];
            end
%             catch;keyboard;end;
            %put them all together
            all_steps=[all_steps,temp4,temp5,temp6];
            
            %get rid of steps where 1,2,3 dont move joint up 4,5,6 before
            %and after steps also include end position (for safety)
            abspathdiff_bin=abs(all_steps(2:end,1:3)-all_steps(1:end-1,1:3))>minjointres;
            all_steps=[all_steps(abspathdiff_bin(:,1)|abspathdiff_bin(:,2)|abspathdiff_bin(:,3),:);
                       all_steps(end,:)];
%             if size(all_steps_temp,1)~=size(all_steps,1)
%                 display(['Size dif is ',num2str(size(all_steps,1)-size(all_steps_temp,1))]);
%             end
%             all_steps=all_steps_temp;
            
            %check for collisions
            if ~isempty(find(all_steps(end,:)-endQ~=0)>0);all_steps=[all_steps;endQ];end
            pathfound=check_path_for_col(all_steps,obsticle_points);
            if pathfound==1; 
%                 all_steps=path_minimise(all_steps);
%                 display('Found a path with combinations');
                return; 
            end
        end        
    end

    %%end additional
    
    
    
    
    
%%Do perms    
    
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
        if pathfound==1; 
%             all_steps=path_minimise(all_steps);
            return; 
        end
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
    
    
    %%additional
    if startQ(1)<0
        %this will put it on the negative side but close to 0
        J1rand=rand()^6*qlimits(1,1);
    elseif startQ(1)>0
        %this will put it on the negative side but close to 0
        J1rand=rand()^6*qlimits(1,2);
    else %startQ(1)==0
        if endQ(1)<0
            J1rand=rand()^6*qlimits(1,1);
        elseif endQ(1)>0
            J1rand=rand()^6*qlimits(1,2);
        else
            J1rand=0;
        end
    end
    middleQ=[J1rand,...
             joint2*(-qlimits(2,1)+qlimits(2,2))+qlimits(2,1),...
             joint3*(-qlimits(3,1)+qlimits(3,2))+qlimits(3,1),...
             startQ(4:6) + (endQ(4:6)-startQ(4:6))/2];

         %commented out this
         %     middleQ=[rand()*(-qlimits(1,1)+qlimits(1,2))+qlimits(1,1),...
%              joint2*(-qlimits(2,1)+qlimits(2,2))+qlimits(2,1),...
%              joint3*(-qlimits(3,1)+qlimits(3,2))+qlimits(3,1),...
%              startQ(4:6) + (endQ(4:6)-startQ(4:6))/2];

    %%%%%end additional
    
    

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

%% FUNCTION: path_minimise
% Go through all paths and if the gradient on one joint goes positive then
% negative remove the middle crap
function all_steps=path_minimise(initial_all_steps)
%intialise return variable
all_steps=initial_all_steps;
initialsize=size(all_steps,1);
%If there is only one value then it is not a path worth checking
if size(all_steps,1)<4
    if size(all_steps,1)==0
        keyboard;
    end;
return;
end

try
    %determine path diferential
    pathdiff=all_steps(2:end,:)-all_steps(1:end-1,:);
    pathdoubdiff=pathdiff(2:end,:)-pathdiff(1:end-1,:);
    tworows_diff0_12=find(pathdiff(:,1)==0&pathdiff(:,2)==0);
    tworows_diff0_23=find(pathdiff(:,2)==0&pathdiff(:,3)==0);
    tworows_diff0_13=find(pathdiff(:,1)==0&pathdiff(:,3)==0);
    if length(tworows_diff0_12)>2 && ...
            ~isempty(find(pathdoubdiff(tworows_diff0_12(1:end-1),3)>minjointres,1)) &&...
            ~isempty(find(pathdoubdiff(tworows_diff0_12(1:end-1),3)<-minjointres,1))
        display('maybe path could be shorter J3');
%             keyboard;
    end
    if length(tworows_diff0_23)>2 &&...
            ~isempty(find(pathdoubdiff(tworows_diff0_23(1:end-1),1)>minjointres,1)) &&...
            ~isempty(find(pathdoubdiff(tworows_diff0_23(1:end-1),1)<-minjointres,1))
        display('maybe path could be shorter J1');
%             keyboard;           
    end
    if length(tworows_diff0_13)>2 &&...
            ~isempty(find(pathdoubdiff(tworows_diff0_13(1:end-1),2)>minjointres,1)) &&...
            ~isempty(find(pathdoubdiff(tworows_diff0_13(1:end-1),2)<-minjointres,1))
        display('maybe path could be shorter J2');    
%         keyboard;
    end
catch
    display('Error in path min func');
    lasterr
    all_steps=initial_all_steps;
    keyboard
end

if initialsize~=size(all_steps,1)
    display(['The path size has been reduced from ',num2str(initalsize),' to ',num2str(size(all_steps,1))]);
end
