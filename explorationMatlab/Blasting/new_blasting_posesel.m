%modified by gavin 090313

%% new_blasting_posesel
%
% *Description:* Now with proper sigmoid functions


%% Function Call
% *Inputs:* 
%
% _pt_ (1*3 double) cartitian point where we want end effector <br>
%
% _plane_equ_ (1*4 double) the desired plane to point at
%
% _q_ (1*6 double) radians - The joint config starting guess 
%
% _showerrors_ (bool) - do we want to show the error vector
%
% *Returns:*
%
% _qt_ (1*6 double) radians - The recommended joint config
%
% _solutionvalid_ (binary) whether the returned qt gives a valid solution
% within optimisation parameters
%
% _dist_val_ (double) Distance between where the plane is hit with blast stream and pt
%
% _targetdist_ (structure) 2 values, (1)Distance between end effector and
% plane, (2) Distance between end effector (nozzel) and pt 


function [qt,solutionvalid,dist_val,targetdist] = new_blasting_posesel(pt, plane_equ, q,showerrors)

if nargin<4
    showerrors=false;
    
end

%showerrors=true;

%this does the extended find with more itterations and starting form all
%zeros
do_longversion=false;
%% Variables
% $$ \begin{array}{lc}
% \mbox{targetNormal...} & V_{tn} \\
% \end{array}$$
    global optimise densoobj workspace r allE;
    allE.E=[];
    allE.Q=[];
    allE.funcE=[];
    allE.funcQ=[];
    
    obstacle_points=workspace.indexedobsticles;
    
    %default is true, unless proven otherwise
    solutionvalid=true;
    
    %display on
    displayon=false;
    maxdistguess=6;
    
    if size(pt, 1) == 1
        pt = pt(:); % make sure pt is a column vector
    end

    %need to normalise plane equation (should be already)
    plane_equ(1:3)=plane_equ(1:3)/norm(plane_equ(1:3));
    
    numlinks = r.n;
    Links = r.link; 
    t = r.base;
    qlimits=r.qlim; 
    L = r.link;
    for piece=1:numlinks
        linkvals(piece).val=[L{piece}.alpha L{piece}.A L{piece}.D L{piece}.offset];
    end
    
    %make sure the q is correct
    if nargin < 3
        q = zeros(numlinks, 1);
    else
        q = q(:);
    end
    
    %save previous q val to see if we need col checking
    previousq=inf*ones([size(q,1),1]);
    temp_prev=q;

    %this is the result of a collision test and it is held for each joint
    result_row=ones([1,size(q,1)]);
    
%% check_that_guess_is_valid
    q=check_that_guess_is_valid(q,qlimits,pt,t,Links,numlinks,plane_equ,displayon,maxdistguess,linkvals);

    
%% Do the least squared optimisation function using q passed in       
history.gradient = [];
history.residual = [];
history.x = [];
    
resnorm2=inf;
q_original=q;
usingzeros=false;
% options = optimset('Display', 'off', 'Largescale', 'off', 'TolFun', optimise.stol,'MaxFunEvals', optimise.iLimit,'DiffMinChange',0.01*pi/180);
    xGuess = zeros([size(q,1)-1,size(q,2)]);lb = []; ub = [];

%% Delete
%     ops=optimset('outputfcn', @outfun,'Display', 'off', 'Largescale', 'off', 'TolFun', optimise.stol,'MaxIter', optimise.iLimit,'MaxFunEvals',optimise.funLimit); 
%     [dq, resnorm1,residual1] = lsqnonlin(@costComponents, xGuess, lb, ub, ops);        
ops=optimise.options;
ops.OutputFcn=@outfun;
[dq, resnorm1,residual1] = lsqnonlin(@costComponents, xGuess, lb, ub, ops);        


    %if some of the residuals are greater than 1 (eg unacceptable) do more itterations
    if ~isempty(find(residual1>1,1)) && do_longversion
%         display('Having to try additional iterations')
        optimise.options.iLimit=4*optimise.iLimit;
        [dq, resnorm1,residual1] = lsqnonlin(@costComponents, xGuess, lb, ub, optimise.options);
                
        %if it is still not valid try to set the starting guess to be all 0s and try long version
        if ~isempty(find(residual1>1,1))  
%             display('trying starting from all zeros')
            q = zeros(numlinks, 1);            
            q=check_that_guess_is_valid(q,qlimits,pt,t,Links,numlinks,plane_equ,displayon,maxdistguess,linkvals);
            [dq, resnorm2,residual2] = lsqnonlin(@costComponents, xGuess, lb, ub, optimise.options);
            usingzeros=true;
        end
        
        optimise.options.iLimit=optimise.iLimit/4;
        
    end
    
    %old way with hardcoded options
%     options = optimset('Display', 'off', 'Largescale', 'off', 'TolFun', 1e-13,'MaxFunEvals', 500);
%     xGuess = zeros([size(q,1)-1,size(q,2)]);lb = []; ub = [];
%     [dq, resnorm2,residual2,exitflag2] = lsqnonlin(@costComponents, xGuess, lb, ub, options);

    % Update the configuration
    if resnorm1<resnorm2
        qt = [q_original(1:5)' + real(dq(1:5))',q_original(6:end)'];
    else %we are using the all 0s start point
        qt = [q(1:5)' + real(dq(1:5))',q(6:end)'];
    end
    
    %if there is a collision or out of joint limit then no distance is returned
    dist_val=inf;    
        
%% Check the optimisation results,
    try [solutionvalid,dist_val,targetdist.val]=classunkcheck_newQ(qt,qlimits,pt,t,Links,numlinks,plane_equ,displayon,linkvals); 
    catch; keyboard; end
 
    if usingzeros && solutionvalid
        display('found one');
    end
      
%% record search path (from steves blastConfiguration)
    function stop = outfun(x,optimValues,state)
        stop = false;
        
%         keyboard
        
        switch state
            case 'init'
%                 hold on
            case 'iter'
                if size(history.residual,2) > 1
                    fval1 = norm(optimValues.residual);
                    fval2 = norm(history.residual(:,end));
                    if abs(fval1 - fval2) < optimise.options.TolFun
                        stop = true;
                    end
                end
                % Concatenate current point and objective function
                % value with history. x must be a row vector.
                history.gradient = [history.gradient optimValues.gradient];
%                 history.fval = [history.fval optimValues.fval];
                history.residual = [history.residual optimValues.residual];
                history.x = [history.x x];

                allE.E=[allE.E,optimValues.residual];
                allE.Q=[allE.Q,[q(1:5)+real(x(1:5));0]];

            case 'done'
%                 hold off
            otherwise
        end
    end
%% EMBEDDED FUNCTION: Run iteratively changing delta q (dq)
    function [e]=costComponents(dq)
        tr=t;               
        
        if ~isempty(find(isnan(dq(1:5))==1,1))
            error('Nan found');
        end
        
        q_temp=[q(1:5)+real(dq(1:5));q(6:end)];
        
        if isempty(find(abs((previousq-q_temp))>pi/180,1))
            skipcollision_check=true;
        else
            skipcollision_check=false;
        end
            
%         result_row=1000*ones([1,size(q_temp,1)]);
        %check each joint and link for collisions and exceeding limits
        for i=1:numlinks; 
%             tr = tr * Links{i}(q_temp(i));
            tr = tr * linktransform_gp(linkvals(i).val,(q_temp(i)));
            if i>2 && i<7 && ~skipcollision_check  && result_row(i)<=1
                try %tempresult=check_FF(tr,densoobj(i+1).ellipse,obstacle_points);
%                     keyboard 
                translated_points_1=obstacle_points(:,1)-tr(1,4);
                translated_points_2=obstacle_points(:,2)-tr(2,4);
                translated_points_3=obstacle_points(:,3)-tr(3,4); 
    
%                 translated_points_1_t=translated_points_1*tr(1,1)+translated_points_2*tr(2,1)+translated_points_3*tr(3,1);
                translated_points_1_t=translated_points_1*tr(1,1)+translated_points_2*tr(1,2)+translated_points_3*tr(1,3);               
%                 translated_points_2_t=translated_points_1*tr(1,2)+translated_points_2*tr(2,2)+translated_points_3*tr(3,2);        
                translated_points_2_t=translated_points_1*tr(2,1)+translated_points_2*tr(2,2)+translated_points_3*tr(2,3);                      
%                 translated_points_3_t=translated_points_1*tr(1,3)+translated_points_2*tr(2,3)+translated_points_3*tr(3,3);   
                translated_points_3_t=translated_points_1*tr(3,1)+translated_points_2*tr(3,2)+translated_points_3*tr(3,3);                   
    
                [tempresult]=min(((translated_points_1_t-densoobj(i+1).ellipse.center(1))/densoobj(i+1).ellipse.params(1)).^2+...
                                 ((translated_points_2_t-densoobj(i+1).ellipse.center(2))/densoobj(i+1).ellipse.params(2)).^2+...
                                 ((translated_points_3_t-densoobj(i+1).ellipse.center(3))/densoobj(i+1).ellipse.params(3)).^2);
                                       
                
                catch tempresult=1000;
                end                
                    
                result_row(i)=tempresult;
            end
        end
        %save the last q when we did a collision check
        if ~skipcollision_check
            previousq=q_temp;
%         else
%             display(['Skipping check because qtemp', num2str(q_temp'),'too close to previous pose',num2str(previousq')]);
        end

%% Error Vector (being minimised) in embedded cost calc function
% $$e=\left( \begin{array}{c}
% e^{4( \sqrt{(P_{dt,x}-P_{at,x})^2+(P_{dt,y}-P_{at,y})^2+(P_{dt,z}-P_{at,z})^2}-C_{maxtargetdis})}\\
% e^{4( \sqrt{(P_{z,x}-P_{az,x})^2+(P_{z,y}-P_{az,y})^2+(P_{z,z}-P_{az,z})^2}-C_{maxtargetdis})}\\
% e^{10( dist_pt2tr(pt,tr)-C_{minAccepDis})}\\
% \frac{e^{5( |Q_{5th}|-C_{maxDeflectionError})}}{100}\\
% e^{5\times \sum{Jlimitresult}}-1\\
% e^{5(6-\sum{result\_row}}-1\\
% \end{array} \right)$$
    
    % Distance from pt aimed at to end effector
    streamlength=dist_pt2tr(pt,tr);
    %Angle between ray and plane 
    theta = acos(plane_equ(1:3)*unit(tr(1:3,3)));
    theta(theta>pi/2)=pi-theta(theta>pi/2);

%      streamStart=tr(1:3,4);
     streamEnd=tr(1:3,4)'+tr(1:3,3)';
     streamEndOp=tr(1:3,4)'-tr(1:3,3)';
%     r_var=[streamStart(1)-streamEnd(1),streamStart(2)-streamEnd(2),streamStart(3)-streamEnd(3)];
    r_var=-tr(1:3,3)';

    %find intersection point between surface and the ray between sensing origin and point
    bottomof_t_var=plane_equ(1)*r_var(1)+...
                   plane_equ(2)*r_var(2)+...
                   plane_equ(3)*r_var(3);
    %make sure it is not 0 otherwise change it so it is simply a very small
    %number (epsilon)
    if ~isempty(find(bottomof_t_var==0, 1)); bottomof_t_var(bottomof_t_var==0)=eps; end                                                                               
    t_var=( plane_equ(1)*tr(1,4)+...
            plane_equ(2)*tr(2,4)+...
            plane_equ(3)*tr(3,4)+...
            plane_equ(4)...
           )./ bottomof_t_var;                 

    % Get the intersection points
    intersectionPNT=[t_var.*-r_var(1)+tr(1,4),...
                     t_var.*-r_var(2)+tr(2,4),...
                     t_var.*-r_var(3)+tr(3,4)];
     
     viewpoint_acuracy=sqrt((pt(1)-intersectionPNT(1))^2+(pt(2)-intersectionPNT(2))^2+(pt(3)-intersectionPNT(3))^2);
     dist_to_stream_end=sqrt((intersectionPNT(1)-streamEnd(1))^2+(intersectionPNT(2)-streamEnd(2))^2+(intersectionPNT(3)-streamEnd(3))^2);
     dist_to_stream_OPend=sqrt((intersectionPNT(1)-streamEndOp(1))^2+(intersectionPNT(2)-streamEndOp(2))^2+(intersectionPNT(3)-streamEndOp(3))^2);
               

k=1;%            k=10;
        e = [1/(1+exp(-k*(theta-optimise.maxDeflectionError))); %g_1
             1- 1/(1+exp(-k*(10*streamlength-10*optimise.mintargetdis)))+1/(1+exp(-k*(10*streamlength-10*optimise.maxtargetdis))); %g_2
             1/(1+exp(-k*(10*viewpoint_acuracy-10*optimise.minAccepDis))); %g_3 (note how optimise.minAccepDis is actually the maximum constraint)
            (1-1/(1+exp(-k*(result_row(3)-1))))^2+...
            (1-1/(1+exp(-k*(result_row(4)-1))))^2+...
            (1-1/(1+exp(-k*(result_row(5)-1))))^2+...
            (1-1/(1+exp(-k*(result_row(6)-1))))^2; %g_4            
            1-1/(1+exp(-k/2*(q_temp(5)-(qlimits(5,1)+pi/12))))+1/(1+exp(-k*(q_temp(5)-(qlimits(5,2)-pi/12)))); %g_5 (using pi/6 as the needed laser range)
             (1-1/(1+exp(-k*(q_temp(1)-qlimits(1,1))))+1/(1+exp(-k*(q_temp(1)-qlimits(1,2)))))^2+...
             (1-1/(1+exp(-k*(q_temp(2)-qlimits(2,1))))+1/(1+exp(-k*(q_temp(2)-qlimits(2,2)))))^2+...
             (1-1/(1+exp(-k*(q_temp(3)-qlimits(3,1))))+1/(1+exp(-k*(q_temp(3)-qlimits(3,2)))))^2+...
             (1-1/(1+exp(-k*(q_temp(4)-qlimits(4,1))))+1/(1+exp(-k*(q_temp(4)-qlimits(4,2)))))^2; %g_6
             1/(1+exp(-k*(dist_to_stream_end-dist_to_stream_OPend))); %dist_to_stream_end-dist_to_stream_OPend %should max at 0
             ]; 
if showerrors 
    e;     
%          dq;
%          temp_prev(1:6)-dq
%          temp_prev=dq;
%           plot(r,q_temp')

allE.funcE=[allE.funcE,e];
allE.funcQ=[allE.funcQ,q_temp];
end






    end
end