%% calc_elip
%
% *Description:* This Function determines the size of this pieces numbers of
% elipses that will cover each of the vertices describing triangles that
% describe the shape

%% Function Call
% 
% *Inputs:* 
%
% _piece_= the current peice that is being plotted
%
% _current_shape_ = (verts) the vertices of the triangles of the shape being plotted
%
% *Returns:* 
%
% _{x,y,z}_ = Cartesian points describing ellispe
%
% _params_ = the parameters a,b,c of elispoid
%
% _center_ = center of ellipse

function [x,y,z,params,center]=calc_elip(piece,current_shape,safetyfactor)

%% Variables
%
% $$center= \frac{\max (point) + \min (point)}{2}$$
%center of elipsoid
center=(max(current_shape)+min(current_shape))/2;

%added safety factor if nothing passed
if nargin<3    
    safetyfactor=0.1;    
end
    
%%
% $$params_{start}= \frac{\max (point) - \min (point)}{2}$$
%the parameters a,b,c of elispoid
params=(max(current_shape)-min(current_shape))/2;

%% Ellipse Function:
% $$ \frac{(x-center_x)^2}{a^2}+\frac{(y-center_y)^2}{b^2}+\frac{(z-center_z)^2}{c^2}-1=0$$

%we want to grow a, b,c
%alternane between growing methods
alternate=0; %intial_params=params;
while  find(((current_shape(:,1)-center(1)).^2)/params(1)^2+...
            ((current_shape(:,2)-center(2)).^2)/params(2)^2+...
            ((current_shape(:,3)-center(3)).^2)/params(3)^2>1)
    %this one increases each param by 1%
    if alternate==0
        params=params*1.01;

        alternate=1;
    else  
        %this one works out the average of a,b,c and then adds 1% of this average to each one
        params=params+sum(params)/3*0.01;
        alternate=0;
    end
end

% p=1.6075;
% SA=4*pi*(((params(1)*params(2))^p+(params(1)*params(3))^p+(params(3)*params(2))^p)/3)^(1/p);
% display(['exact SA = ',num2str(SA)]);
% display(['exact volume = ',num2str(4/3*pi*params(1)*params(2)*params(3))]);

params=params*(1+safetyfactor);
% SA=4*pi*(((params(1)*params(2))^p+(params(1)*params(3))^p+(params(3)*params(2))^p)/3)^(1/p);
% display(['exact SA = ',num2str(SA)]);
% display(['volume *1.02 = ',num2str(4/3*pi*params(1)*params(2)*params(3))]);

params=params+sum(params)/3*safetyfactor;
% SA=4*pi*(((params(1)*params(2))^p+(params(1)*params(3))^p+(params(3)*params(2))^p)/3)^(1/p);
% display(['exact SA = ',num2str(SA)]);
% display(['volume + sumofall/3 by 0.02 = ',num2str(4/3*pi*params(1)*params(2)*params(3))]);
        
%% Matlab's Ellipse Shell Creation Function
[x,y,z]=ellipsoid(center(1),center(2),center(3),params(1),params(2),params(3),10);

%display(strcat('For piece:',num2str(piece),',The center is at',num2str(center),', the a,b,c params are',num2str(params)));
%display(strcat('initial params were',num2str(intial_params),'new ones are',num2str(params)));

