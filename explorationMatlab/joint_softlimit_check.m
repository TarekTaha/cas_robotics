%% joint_softlimit_check
%
% *Description:*  Checks a formula which specifies whether this
% configuration is going to result in going out of the soft limits of the
% arm 

%% Function Call
%
% *Inputs:* 
%
% _all_steps_ (6*m double) these are the joint configs radians
%
% *Returns:* 
% 
% _result_ (bin) =1 if ok, =0 if NOT ok

function result=joint_softlimit_check(all_steps)

%% Variables

% Result is intially set to true
result=true;

%% Change all steps to degs since eeasier to work with
all_steps_indegs=rad2deg(all_steps);

%% Check all_steps
% $$ \begin{array}{rl}
% \forall Q \in all\_steps: &
% Q\times\frac{180}{\pi}\le \frac{-7}{3} (Q\times\frac{180}{\pi}) -130 \\
% \rightarrow & \mbox{ELSE exceeds soft limit(return 0)}
% \end{array}$$
if ~isempty(find(all_steps_indegs(:,3)<-7/3*all_steps_indegs(:,2)-130,1))
    result=false;
    return
end

if ~isempty(find(all_steps_indegs(:,2)>90 & all_steps_indegs(:,3)>90,1))
    result=false;
    return
end

%% if we have some poses with J3>140' Check new EE piece for self collision
if ~isempty(find(all_steps_indegs(:,3)>140,1)) 
  %get the steps we need to test
  J4between90s=find(all_steps_indegs(:,4)<=90 & all_steps_indegs(:,4)>=-90 & all_steps_indegs(:,3)>140);
  %the polyfit vals I collected from experimentation (for J4 between -90 and 90)
  if size(J4between90s,1)>0
    
    polyC=[.64935058504714284724652906266584e-5,...
           -.59625596809581623419793672269407e-2,...
           2.2796457041226108053422194643645,...
           -464.50865585679287050879793241620,...
           53203.113286802014044951647520065,...
           -3247693.4326907582581043243408203,...
           82546906.916412547230720520019531];


    if ~isempty(find(polyC(1)*all_steps_indegs(J4between90s,3).^6+...
                     polyC(2)*all_steps_indegs(J4between90s,3).^5+...
                     polyC(3)*all_steps_indegs(J4between90s,3).^4+...
                     polyC(4)*all_steps_indegs(J4between90s,3).^3+...
                     polyC(5)*all_steps_indegs(J4between90s,3).^2+...
                     polyC(6)*all_steps_indegs(J4between90s,3).^1+...
                     polyC(7)<all_steps_indegs(J4between90s,5),1))
      result=false;
      return           
    end
  end
  
  %check if J4 is out of 90 degs  
  J4outof90s=find((all_steps_indegs(:,4)>90 | all_steps_indegs(:,4)<-90) & all_steps_indegs(:,3)>140);
  if size(J4outof90s,1)>0
    %the polyfit vals I collected from experimentation (for J4<-90 ||
    %J4>90)
    polyC=[ .70203186009616268768273806063420e-15,...
        .33333333281924671905385726411453e-3,...
        -.20333333318296215042053631805175,...
        46.391666644705857436292717466131,...
        -4689.9166650653205579146742820740,...
        177114.99995335948187857866287231];
      
    if ~isempty(find(polyC(1)*all_steps_indegs(J4outof90s,3).^5+...
                     polyC(2)*all_steps_indegs(J4outof90s,3).^4+...
                     polyC(3)*all_steps_indegs(J4outof90s,3).^3+...
                     polyC(4)*all_steps_indegs(J4outof90s,3).^2+...
                     polyC(5)*all_steps_indegs(J4outof90s,3).^1+...
                     polyC(6)>all_steps_indegs(J4outof90s,5),1))
    result=false;
    return           
    end
  end
end
  
%% Experimental data of edges use to get polyC
% close all
% % if J3>140
% %   if J4<90 || J4>-90   
%   figure(1)
%   J3=[165,162,160,155,150,145,143,140];
%   J5=[55,60,65,70,80,90,95,105];
%   plot(J3,J5,'r')
%   polyC=polyfit(J3,J5,6);
%   xvals=[min(J3):max(J3)];
% %   yvals=polyC(1)*xvals.^5+...
% %         polyC(2)*xvals.^4+...
% %         polyC(3)*xvals.^3+...
% %         polyC(4)*xvals.^2+...
% %         polyC(5)*xvals.^1+...
% %         polyC(6);
%   yvals=polyC(1)*xvals.^6+...
%         polyC(2)*xvals.^5+...
%         polyC(3)*xvals.^4+...
%         polyC(4)*xvals.^3+...
%         polyC(5)*xvals.^2+...
%         polyC(6)*xvals.^1+...
%         polyC(7);
% 
%   hold on;
%   plot(xvals,yvals,'b')
% 
% 
% %   elseif J4<90 || J4>-90
% 
% figure(2)
% 
% J3=[165,160,155,150,145,140]
% J5=[-40,-45,-50,-60,-75,-90]
% plot(J3,J5,'r')
% polyC=polyfit(J3,J5,5);
% xvals=[min(J3):max(J3)];
% yvals=polyC(1)*xvals.^5+...
%       polyC(2)*xvals.^4+...
%       polyC(3)*xvals.^3+...
%       polyC(4)*xvals.^2+...
%       polyC(5)*xvals.^1+...
%       polyC(6);
% hold on;
% plot(xvals,yvals,'b')
% %   end
% % end

%% New Proposed approach
%it seems that this 'new' approach has actually been done above before
% 
% p3n4 = 1.0e+003 *[   0.0000   -0.0172    1.6054];
% p3n5 = 1.0e+003 *[  -0.0002    0.0577   -4.6839];
% 
% step1=find(all_steps_indegs(:,3)>140);
% if ~isempty(step1) & false
% %% do j5 positive
% %   positive_j5=find(all_steps_indegs(step1,5)>0);
%   
%   
% %% do j5 negative
%   negative_j5=find(all_steps_indegs(step1,5)>0);
%   j4_limit = polyval(p3n4,deg2rad(all_steps_indegs(step1(negative_j5))),3);  
%   %if there are any abs j4s grewater than the combinational boundary
%   if ~isempty(find(abs(all_steps_indegs(step1(negative_j5),4))>j4_limit,1))
%     result=false;
%     return
%   end
%   
%   j5_limit = polyval(p3n5,deg2rad(all_steps_indegs(step1(negative_j5))),3);
%   %if there are any abs j5s less than the combinational boundary
%   if ~isempty(find(all_steps_indegs(step1(negative_j5),5)<j5_limit,1))
%     result=false;
%     return
%   end 
% end