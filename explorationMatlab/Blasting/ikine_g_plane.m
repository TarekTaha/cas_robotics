%ikine based upon P Cook's robot toolbox
%modified for speed
function q=ikine_g_plane(r,pt, plane_equ,q,linkvals)
global optimise
tr=zeros([4,4]);
tr(4,4)=1;

%try and point away from center
if norm(pt-plane_equ(1:3))<norm(pt+plane_equ(1:3))
    tr(1:3,3)=[plane_equ(1:3)]';   
    tr(1:3,4)=[pt-[plane_equ(1:3).*optimise.maxtargetdis]]';
else
    tr(1:3,3)=[-plane_equ(1:3)]';   
    tr(1:3,4)=[pt+[plane_equ(1:3).*optimise.maxtargetdis]]';
end
  
%limit variables
stol = 1e-4;
ilimit = 50;

% Start variables
deltaq_normalised=inf;
q=q(1:6);
q=q(:);


% robot variables
n = r.n;
rob_base = r.base;
L = r.link;
LinkTrans=zeros([4,n*4]);
if nargin<5
    for piece=1:n
        linkvals(piece).val=[L{piece}.alpha L{piece}.A L{piece}.D L{piece}.offset];
    end
end
rob_tool=r.tool*linktransform_gp(linkvals(7).val,(0));
%only go through 6 links
n=6;

%% main loop
while deltaq_normalised>stol
    %determine the link tranform mats for each piece
    for piece=1:n; LinkTrans(:,4*piece-3:4*piece)=linktransform_gp(linkvals(piece).val,(q(piece)));end
    %fkine
    Tn = rob_base; for piece=1:n; Tn = Tn * LinkTrans(:,4*piece-3:4*piece); end;
    %difference between new transfrom (Tn) and desired transfrom (tr)
    e=diff2tr(Tn,tr);
    if 0.2 < norm(e(1:3))
        e(1:3) = 0.2 * e(1:3) / norm(e(1:3));
    end
    %if the positional error norm(e(4:6))is greater than 0.2
    if 0.2 < norm(e(4:6))
        e(4:6) = 0.4472 * e(4:6) / norm(e(4:6));
    end

    %based upon jacobn function 
    %parameters to be reset
    Jn = zeros([6,n]);
    U = rob_tool;
    %go through joints in inverse
    for piece=n:-1:1,
        % standard DH convention
        U = LinkTrans(:,4*piece-3:4*piece) * U;
        % revolute axis joints ONLY
        d = [	-U(1,1)*U(2,4)+U(2,1)*U(1,4)
				-U(1,2)*U(2,4)+U(2,2)*U(1,4)
				-U(1,3)*U(2,4)+U(2,3)*U(1,4)];
		Jn(:,piece) = [d; U(3,1:3)'];
    end
    
	jacob_temp = [Tn(1:3,1:3) zeros(3,3); zeros(3,3) Tn(1:3,1:3)] * Jn;
          
    dq = pinv(jacob_temp')' * e;
        
    q = q + dq;
    %add the change to the current q
    q(q>=0)=mod(q(q>=0),2*pi);
    q(q<0)=mod(q(q<0),-2*pi);
    
    deltaq_normalised =norm(dq);    
        
    %check if we have done too many itterations
    if ilimit==0; break        
    else; ilimit=ilimit-1;
    end
end
%make into correct format
q=q(:)';
q=[q,0];


%% vector of difference between 2 transforms
function d=diff2tr(t1,t2)
d = [t2(1:3,4)-t1(1:3,4);
     0.5*(cross_g(t1(1:3,3), t2(1:3,3)))];
        
%% my cross product
function c=cross_g(a,b)    
c = [a(2,:).*b(3,:)-a(3,:).*b(2,:)                  
     a(3,:).*b(1,:)-a(1,:).*b(3,:)                 
     a(1,:).*b(2,:)-a(2,:).*b(1,:)];