%This code plots the robot with the ideal spray location + pose selection
%spray location
%Generate the ikine now for testing purpose
% load tempplane
% setupoptimisation
% plane = tempplane(25);
function poseVisualisationRTA(plane)
r = rob_object;

for i=1:size(plane,2)
    t = fkine(r, plane(i).Q);
    plot(r,plane(i).Q,'axis','equal');
    hold on
    axis equal
    
    sprayPath(plane(i),0.04,1,'r')
    try delete(h1);delete(h2);delete(h3); end;
    h1 = line([plane(i).home_point(1) (plane(i).equ(1)+plane(i).home_point(1))],...
    [plane(i).home_point(2) (plane(i).equ(2)+plane(i).home_point(2))],...
    [plane(i).home_point(3) (plane(i).equ(3)+plane(i).home_point(3))],'Marker','.','LineStyle','--')
    targetNormal = t(:,3) + t(:,4);
    h2=line([targetNormal(1) t(1,4)], [targetNormal(2) t(2,4)], [targetNormal(3) t(3,4)],'Marker','+','LineStyle','-')
    org = targetNormal(1:3)';
    dir = (t(1:3,4)-targetNormal(1:3))';
    dirInverse = (targetNormal(1:3)-t(1:3,4))';
    k = plane(i).equ(4);
    normal = plane(i).equ(1:3);
    u = (plane(i).equ(4) + dot(plane(i).equ(1:3), targetNormal(1:3)'))/...
            (dot(plane(i).equ(1:3),(targetNormal(1:3)-t(1:3,4))'));
    line2 = org + u*dir;
    d = sqrt((plane(i).home_point(1) - line2(1))^2 +...
            (plane(i).home_point(2) - line2(2))^2 +...
            (plane(i).home_point(3) - line2(3))^2);
     CoordinateDistance = line2 - plane(i).home_point;
     ShiftedNormal = CoordinateDistance + plane(i).equ(1:3) + plane(i).home_point(1:3);
     h3=line([line2(1) ShiftedNormal(1)],...
        [line2(2) ShiftedNormal(2)],...
        [line2(3) ShiftedNormal(3)],'Marker','.','LineStyle','-')
    extendedline2 = line2 + t(1:3,3)';
    testing1 = dot(plane(i).equ(1:3), t(1:3,3)');
    testing2 = dot((extendedline2-line2), (ShiftedNormal-line2));
    CosB = dot(ShiftedNormal, t(1:3,4)');
    Angle = acos((180/pi)*dot(t(1:3,1), t(1:3,2)));
    hold on
    plot3(line2(1), line2(2), line2(3),'o');
    plot3(extendedline2(1), extendedline2(2), extendedline2(3),'o');
    %plot(r,plane(i).Q,'axis','equal');
end


