%% robot_transform_matrix
function tr=robot_transform_matrix(newQ,linkvals_quick,numjoints_to_go_through)
    if nargin<3
        numjoints_to_go_through=length(newQ);
    end
    an = linkvals_quick(:,1);
    dn = linkvals_quick(:,2);
    theta = linkvals_quick(:,3)+newQ(:);
    sa = linkvals_quick(:,4); 
    ca = linkvals_quick(:,5);

    tr=zeros([4,4,numjoints_to_go_through],'single');

    tr(:,:,1)=linktransform_quick_priv(an(1),...
                                       dn(1),...
                                       theta(1),...
                                       sa(1),...
                                       ca(1));                                

    for curr_joint=2:numjoints_to_go_through                                 
        tr(:,:,curr_joint)=tr(:,:,curr_joint-1)*linktransform_quick_priv(an(curr_joint),...
                                                                         dn(curr_joint),...
                                                                         theta(curr_joint),...
                                                                         sa(curr_joint),...
                                                                         ca(curr_joint));                                                                                            
    end
end
    
%% linktransform_quick
function t=linktransform_quick_priv(an,dn,theta,sa,ca)
%% Function Call
%
% *Inputs:* 
%
% _a_ (4*1 double) [L{linknun}.alpha,L{linknun}.a,L{linknun}.d,L{linknun}.offset] from robot
%
% _b_ (double) angle in radians to be rotated
%
% *Returns:* 
%
% _t_ (4*4 double) homogeneous transform


st = sin(theta); 
ct = cos(theta);

% return the transform
t =    [ct	-st*ca	st*sa	an*ct
        st	ct*ca	-ct*sa	an*st
        0	sa	ca	dn
        0	0	0	1];
end