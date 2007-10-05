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

%% Check all_steps
% $$ \begin{array}{cc}
% \forall Q \in all\_steps: &
% Q\times\frac{180}{\pi}\le \frac{-7}{3} (Q\times\frac{180}{\pi}) -130
% \end{array}$$
if ~isempty(find(all_steps(:,3)*180/pi<-7/3*all_steps(:,2)*180/pi-130,1))
    result=false;
end