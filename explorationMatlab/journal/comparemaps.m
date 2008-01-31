function [u,sig,D] = comparemaps(patch)
% close all
testnum=11;
display('This always set to test num 11 in comparemaps!');
mew=0.05;
% patch=9;

%% WEB
if patch==1
%left web front
IntPs=[0.50,-0.65,1.25];
%this one shows the pipe but not a plane    
% IntPs=[0.25,-0.6,1.22];
%right web front
elseif patch==2
    IntPs=[0.25,0.6,1.23];
%left web back
elseif patch==3
    IntPs=[-0.25,-0.6,1.22];
%right web back (HAD TO BE MOVED FORWARD this is where the girder is at the back so it must be brought forward)
elseif patch==4
    IntPs=[-0.05,0.6,1.23];
%% ROOF    
%right roof front 
elseif patch==5
    IntPs=[0.25,0.17,1.5];    
%right roof back HAD TO BE MOVED FORWARD this is where the girder is at the back so it must be brought forward) 
elseif patch==6 
    IntPs=[-0.10,0.17,1.5];    
%left roof front
elseif patch==7
    IntPs=[0.25,-0.25,1.5];    
%left roof back
elseif patch==8
    IntPs=[-0.20,-0.25,1.5];
%% FLANGE
%right bottom flange %HAVE TO MOVE since it was still getting the wall
elseif patch==9
    IntPs=[0,0.4,0.85];    
%left bottom flange
elseif patch==10
    IntPs=[0,-0.68,0.8];    
end

if patch<5
    sizeInt=0.21;
elseif (patch <9 && patch >=5)
    sizeInt=0.16;
else
    sizeInt=0.20;
end
    
PlanesofInt=[];
figure(1)
load(['test',num2str(testnum),'hMesh.mat']);
% plot3(hMeshdata.v(:,1),hMeshdata.v(:,2),hMeshdata.v(:,3),'b','marker','.','linestyle','none','markersize',0.05);axis equal; grid on;hold on

cd ..
index=GetImpLevInfo(hMeshdata.v);
index2=find(sqrt((hMeshdata.v(index,1)-IntPs(1)).^2+...
                 (hMeshdata.v(index,2)-IntPs(2)).^2+...
                 (hMeshdata.v(index,3)-IntPs(3)).^2)<sizeInt);
% index2=find(sqrt((hMeshdata.v(index,1)-IntPs(1)).^2+...
%                  (hMeshdata.v(index,2)-IntPs(2)).^2+...
%                  (hMeshdata.v(index,3)-IntPs(3)).^2)<sizeInt | sqrt((hMeshdata.v(index,1)-IntPs1(1)).^2+...
%                  (hMeshdata.v(index,2)-IntPs1(2)).^2+...
%                  (hMeshdata.v(index,3)-IntPs1(3)).^2)<sizeInt);             
plot3(hMeshdata.v(index(index2),1),hMeshdata.v(index(index2),2),hMeshdata.v(index(index2),3),'r','marker','.','linestyle','none','markersize',0.1);axis equal; grid on


cd journal
brSec=plyread('bridgeSection.ply');

hold on
 plot3(brSec.vertex.x,brSec.vertex.y,brSec.vertex.z,'g*')
axis equal

points=hMeshdata.v(index(index2),:);
if patch==1 || patch==3
%first plot (left wall)
p1=[brSec.vertex.x(11),brSec.vertex.y(11),brSec.vertex.z(11)];%11
p2=[brSec.vertex.x(12),brSec.vertex.y(12),brSec.vertex.z(12)];%12
p3=[brSec.vertex.x(15),brSec.vertex.y(15),brSec.vertex.z(15)];%15
elseif patch==2 ||patch==4
% secondplot (right wall)
p1=[brSec.vertex.x(42),brSec.vertex.y(42),brSec.vertex.z(42)];
p2=[brSec.vertex.x(41),brSec.vertex.y(41),brSec.vertex.z(41)];
p3=[brSec.vertex.x(46),brSec.vertex.y(46),brSec.vertex.z(46)];   

elseif patch==5 || patch==6
 % right roof
p1=[brSec.vertex.x(56),brSec.vertex.y(56),brSec.vertex.z(56)];
p2=[brSec.vertex.x(55),brSec.vertex.y(55),brSec.vertex.z(55)];
p3=[brSec.vertex.x(58),brSec.vertex.y(58),brSec.vertex.z(58)];

elseif patch==7 || patch==8
 % left roof 28 1 19
p1=[brSec.vertex.x(28),brSec.vertex.y(28),brSec.vertex.z(28)];
p2=[brSec.vertex.x(27),brSec.vertex.y(27),brSec.vertex.z(27)];
p3=[brSec.vertex.x(19),brSec.vertex.y(19),brSec.vertex.z(19)];

elseif patch==9
 % left flange
% p1=[brSec.vertex.x(37),brSec.vertex.y(37),brSec.vertex.z(37)];
% p2=[brSec.vertex.x(40),brSec.vertex.y(40),brSec.vertex.z(40)];
% p3=[brSec.vertex.x(38),brSec.vertex.y(38),brSec.vertex.z(38)];
p1=[brSec.vertex.x(33),brSec.vertex.y(33),brSec.vertex.z(33)];
p2=[brSec.vertex.x(34),brSec.vertex.y(34),brSec.vertex.z(34)];
p3=[brSec.vertex.x(35),brSec.vertex.y(35),brSec.vertex.z(35)];


elseif patch==10
 % right flange
 p1=[brSec.vertex.x(3),brSec.vertex.y(3),brSec.vertex.z(3)];
p2=[brSec.vertex.x(1),brSec.vertex.y(1),brSec.vertex.z(1)];
p3=[brSec.vertex.x(2),brSec.vertex.y(2),brSec.vertex.z(2)];


end


temp=[p1;p2;p3];
plot3(temp(:,1),temp(:,2),temp(:,3),'y');
axis equal

norm=cross((p2-p1),(p3-p1));
%d=-ax-by-cz
d=-norm(1)*p1(1)-norm(2)*p1(2)-norm(3)*p1(3);
plane_equ=[norm,d];

planesign=(plane_equ(1)*points(:,1)+...
           plane_equ(2)*points(:,2)+...
           plane_equ(3)*points(:,3)+...
           plane_equ(4))>0;
       planesign=2*planesign-1;
 figure;
 hist(planesign)
 
D=abs(plane_equ(1)*points(:,1)+plane_equ(2)*points(:,2)+plane_equ(3)*points(:,3)+plane_equ(4)*ones([size(points,1),1]))./...
    sqrt(plane_equ(1)^2+plane_equ(2)^2+plane_equ(3)^2);
D=D.*planesign;
figure;
hist(D,50)


[u,sig]=normfit(D);
Xrang=[-0.05:0.001:0.05];Yrang = pdf('norm',Xrang,u,sig);

hold on;plot(Xrang,Yrang)

%do a box plot
figure
boxplot(D)


% surface_making_simple(points,mew)
% cd journal

% global plane
% 
% all_home_points=[];
% all_norms=[];
% 
% 
% 
% for i=1:size(plane,2)
%     if sqrt((plane(i).home_point(1)-IntPs(1))^2+(plane(i).home_point(2)-IntPs(2))^2+(plane(i).home_point(3)-IntPs(3))^2)<sizeInt        
%         all_home_points=[all_home_points;plane(i).home_point];
%         all_norms=[all_norms;plane(i).equ(1:3)];    
%         PlanesofInt=[PlanesofInt;i];
%     end
% end
% newplane=plane(PlanesofInt);
% plot_planes(newplane,mew);
% 
% 
% cd journal
% 
% %just pretend the first one is correct
% 
% plane_equ=plane(1).equ;
% % all_home_points
% 
% planesign=(plane_equ(1)*all_norms(:,1)+...
%           plane_equ(2)*all_norms(:,2)+...
%           plane_equ(3)*all_norms(:,3)+...
%           plane_equ(4))>0;
%       
% %Ia=all_home_points
% %Ib=all_norms
% r_var=all_home_points-all_norms;
% 
% %find intersection point between surface and the scan line between scan origin and point
% bottomof_t_var=plane_equ(1)*r_var(:,1)+...
%                plane_equ(2)*r_var(:,2)+...
%                plane_equ(3)*r_var(:,3);
% %make sure it is not 0 otherwise change it so it is simply a very small
% %number (epsilon)
% if ~isempty(find(bottomof_t_var==0, 1)); bottomof_t_var(bottomof_t_var==0)=eps; end                                                                               
% t_var=( plane_equ(1)*all_home_points(:,1)+...
%         plane_equ(2)*all_home_points(:,2)+...
%         plane_equ(3)*all_home_points(:,3)+...
%         plane_equ(4)...
%        )./ bottomof_t_var;                 
% 
% % Get the intersection points
% intersectionPNT=[t_var.*-r_var(1)+all_home_points(:,1),...
%                  t_var.*-r_var(2)+all_home_points(:,2),...
%                  t_var.*-r_var(3)+all_home_points(:,3)];
% 
% %the distance on the plane between where we aimed and where it hit is  
% dist=sqrt((all_home_points(:,1)-intersectionPNT(:,1)).^2+...
%           (all_home_points(:,2)-intersectionPNT(:,2)).^2+...
%           (all_home_points(:,3)-intersectionPNT(:,3)).^2);
%           
%                       
% % for i=1:size(plane,2)  
% %     Ia=all_norms;
% %     Ib=plane_equ(1:3);
% %     
% % %     Ia+(Ib-Ia)t;
% %     r_var=-all_norms(i);
% %     
% % end
% 
% %     r_var=-all_norms(i);
% %     bottomof_t_var=plane_equ(1)*r_var(1)+...
% %                    plane_equ(2)*r_var(2)+...
% %                    plane_equ(3)*r_var(3);
% % 
% %     if ~isempty(find(bottomof_t_var==0, 1)); bottomof_t_var(bottomof_t_var==0)=eps; end                                                                               
% %         t_var=( plane_equ(1)*tr(1,4)+...
% %                 plane_equ(2)*tr(2,4)+...
% %                 plane_equ(3)*tr(3,4)+...
% %                 plane_equ(4)...
% %                )./ bottomof_t_var;                 
% % 
% %         % Get the intersection points
% %         intersectionPNT=[t_var.*-r_var(1)+tr(1,4),...
% %                          t_var.*-r_var(2)+tr(2,4),...
% %                          t_var.*-r_var(3)+tr(3,4)];
% % end
