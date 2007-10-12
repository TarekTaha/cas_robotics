function drawthevrworld()
global r Q myworld
n = r.n;
L = r.link;
tr = r.base;

keyboard

[Rx,Ry,Rz]=determinerots(tr); 
% setfield(myworld.piece1,'translation',[tr(1,4),tr(2,4),tr(3,4)]);
setfield(myworld.piece1,'translation',[tr(2,4),tr(3,4)+0.18,tr(1,4)+0.18]);
% setfield(myworld.piece1,'rotation',[1,0,0,Rx]);
% setfield(myworld.piece1,'rotation',[0,1,0,Ry]);
% setfield(myworld.piece1,'rotation',[0,0,1,Rz]);
setfield(myworld.piece1,'rotation',[1,0,0,Ry]);
setfield(myworld.piece1,'rotation',[0,1,0,Rz]);
setfield(myworld.piece1,'rotation',[0,0,1,Rx]);



tr = tr * L{1}(Q(1));
[Rx,Ry,Rz]=determinerots(tr); 
setfield(myworld.piece2,'translation',[tr(2,4),tr(3,4),tr(1,4)]);
setfield(myworld.piece2,'rotation',[1,0,0,Ry]);
setfield(myworld.piece2,'rotation',[0,1,0,Rz]);
setfield(myworld.piece2,'rotation',[0,0,1,Rx+pi/2]);

tr = tr * L{2}(Q(2));
[Rx,Ry,Rz]=determinerots(tr);
setfield(myworld.piece3,'translation',[tr(2,4),tr(3,4),tr(1,4)]);
setfield(myworld.piece3,'rotation',[1,0,0,Ry]);
setfield(myworld.piece3,'rotation',[0,1,0,Rz]);
setfield(myworld.piece3,'rotation',[0,0,1,Rx]);

tr = tr * L{3}(Q(3));
[Rx,Ry,Rz]=determinerots(tr);
setfield(myworld.piece4,'translation',[tr(2,4),tr(3,4),tr(1,4)]);
setfield(myworld.piece4,'rotation',[1,0,0,Ry]);
setfield(myworld.piece4,'rotation',[0,1,0,Rz]);
setfield(myworld.piece4,'rotation',[0,0,1,Rx]);

tr = tr * L{4}(Q(4));
[Rx,Ry,Rz]=determinerots(tr);
setfield(myworld.piece5,'translation',[tr(2,4),tr(3,4),tr(1,4)]);
setfield(myworld.piece5,'rotation',[1,0,0,Ry]);
setfield(myworld.piece5,'rotation',[0,1,0,Rz]);
setfield(myworld.piece5,'rotation',[0,0,1,Rx]);

tr = tr * L{5}(Q(5));
[Rx,Ry,Rz]=determinerots(tr);
setfield(myworld.piece6,'translation',[tr(2,4),tr(3,4),tr(1,4)]);
setfield(myworld.piece6,'rotation',[1,0,0,Ry]);
setfield(myworld.piece6,'rotation',[0,1,0,Rz]);
setfield(myworld.piece6,'rotation',[0,0,1,Rx]);

vrdrawnow;

%% FUNCTION: determinerots 
% to work out the rotations around the x,y,z to be passed to vrlm
function [Rx,Ry,Rz]=determinerots(tr)    

Rz=atan2(tr(2,1),tr(1,1));
Ry=atan2(-tr(3,1),tr(1,1)*cos(Rz)+tr(2,1)*sin(Rz));
Rx=atan2(-tr(2,3)*cos(Rz)-tr(1,3)*sin(Rz),tr(2,2)*cos(Rz)-tr(1,2)*sin(Rz));

% to_compare=[cos(Rz)*cos(Ry) , cos(Rz)*sin(Ry)*sin(Rx)-sin(Rz)*cos(Rx) , cos(Rz)*sin(Ry)*cos(Rx)+sin(Rz)*sin(Rx);
%             sin(Rz)*cos(Ry) , sin(Rz)*sin(Ry)*sin(Rx)+cos(Rz)*cos(Rx) , sin(Rz)*sin(Ry)*cos(Rx)-cos(Rz)*sin(Rx);
%             -sin(Ry)        , cos(Ry)*sin(Rx)                         , cos(Ry)*cos(Rx)];
% 
% if ~isempty(find(to_compare-tr(1:3,1:3)>eps,1))
%     keyboard
% end
