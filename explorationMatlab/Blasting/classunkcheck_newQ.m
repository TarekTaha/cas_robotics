%% FUNCTION: checks pose validity, if valid but dist>min, return distance 
function [valid,dist,targetdist,correctway,theta]=classunkcheck_newQ(qt,qlimits,pt,tr,Links,numlinks,plane_equ,displayon,linkvals,docollcheck)
    global optimise densoobj workspace;
    if nargin<10
        docollcheck=1;
    end
    %do we want the theta vector for joint 5 to be calculated and passed back
    if nargout==4
    	do_theta=true;
    else
        do_theta=false;
    end
    %returns infinite distance by default
    dist=inf;
    correctway=true;
    theta=inf;
    
    %distance from tr to target pt, distance from tr to target plane
    targetdist=[inf,inf];
    
    %it is not valid by default so we can return at anytime and return invalid
    valid=false;
    

    tr1=tr;
    
    for i=1:numlinks; 
%         tr1 = tr1 * Links{i}(qt(i));
        tr = tr * linktransform_gp(linkvals(i).val,qt(i));
        %if we want to save the tr after the 4th transform for broad theta cals do so here
        if do_theta && i==4
            trfrom4=tr;
        end
            
        if i<=6
            if qt(i)<qlimits(i,1) || qt(i)>qlimits(i,2); 
                if displayon; display(['Outside boundary, Joint i= ',num2str(i)]); end
                return; 
            end;
            if docollcheck
                if i>2 % same as check_path_for_collision 
                    if ~check_FF(tr,densoobj(i+1).ellipse,workspace.indexedobsticles); 
                        if displayon; display(['Collision Detected on densoobj(i+1)=',num2str(i+1)]); end
                        return; 
                    end;
                end
            end
        end
    end
          
    
    % Check the actual distance from end effector to target point
    targetdist(1)=dist_pt2tr(pt,tr);  
    
    %the end stream and the end stream in opposite direction
    streamEnd=tr(1:3,4)'+tr(1:3,3)';
    streamEndOp=tr(1:3,4)'-tr(1:3,3)';

    r_var=-tr(1:3,3);

    %find intersection point between surface and the ray between scan origin and point
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

    targetdist(2)=dist_pt2tr(intersectionPNT,tr);
    
    %the distance on the plane between where we aimed and where it hit is  
    dist=sqrt((pt(1)-intersectionPNT(1))^2+...
              (pt(2)-intersectionPNT(2))^2+...
              (pt(3)-intersectionPNT(3))^2);
    
    %if we want the whole theta vector returned
    if do_theta
        %angle between line (lots of possibilities) and plane
        theta=[];
        for angs=-pi/2:10*pi/180:pi/2
            %only start with the 4th transform
            
%             tr1=trfrom4* Links{5}(angs);
            tr = trfrom4 * linktransform_gp(linkvals(5).val,angs);
            for i=6:numlinks; 
%                 tr1 = tr1 * Links{i}(qt(i));
                tr = tr * linktransform_gp(linkvals(i).val,qt(i));
            end
            theta = [theta;acos(plane_equ(1:3)*unit(tr(1:3,3)))];           
        end
        theta(theta>pi/2)=pi-theta(theta>pi/2);
    else %just do the actual
        theta = acos(plane_equ(1:3)*unit(tr(1:3,3)));
        theta(theta>pi/2)=pi-theta(theta>pi/2);
    end
    
     
    %if it is allowable then it is valid and change this to return
    % note the angle test MUST be first since it checks that it is facing
    % correct way
    if (intersectionPNT(1)-streamEnd(1))^2+(intersectionPNT(2)-streamEnd(2))^2+(intersectionPNT(3)-streamEnd(3))^2>(intersectionPNT(1)-streamEndOp(1))^2+(intersectionPNT(2)-streamEndOp(2))^2+(intersectionPNT(3)-streamEndOp(3))^2
        if displayon; display('Stream is facing the wrong way'); end
        correctway=false;        
        valid=false; 
    elseif targetdist(1)<optimise.mintargetdis
        if displayon; display('End Effector is TOO CLOSE to aimed at point');end
        valid=false;
    elseif targetdist(1)>optimise.maxtargetdis
        if displayon; display('End Effector is TOO FAR AWAY from aimed at point');end
        valid=false;
    elseif dist>optimise.minAccepDis
        if displayon; display('The actual INTERSECTION point shot is TOO FAR AWAY from aimed at point');end
        valid=false;
    elseif isempty(find(theta<optimise.maxDeflectionError,1)) 
        if displayon; display('The angle to the surface is too great');end
        valid=false;   
    else
        %if we get to here it is valid
        valid=true;
%         display('Found a solution');
    end;       
end

