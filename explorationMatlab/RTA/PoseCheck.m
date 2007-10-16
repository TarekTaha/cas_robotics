%Function returns 1 for valid pose and 0 for invalid pose
%checkLevel indicates whether point is an interval, thus relaxing the
%threshold. 1 = real spray points, 2 or greater = interval spray points
function result = PoseCheck(plane, checkLevel)
    global r ;
    t = fkine(r, plane.Q);
    plotdenso(r,plane.Q);
    result = 0;
    targetNormal = (t(:,3) + t(:,4))';
    % Work out the value of 'u' for the intersection of the line and plane 
    u = (plane.equ(4) + dot(plane.equ(1:3), targetNormal(1:3)))/...
        (dot(plane.equ(1:3),(t(1:3,3)')));
    %Coordinate of intersection between plane and line
    intersection = targetNormal(1:3) + u*(t(1:3,4)'-targetNormal(1:3));
    %Find the distance between homepoint and intersection
    %keyboard
    d = sqrt((plane.home_point(1) - intersection(1))^2 +...
                (plane.home_point(2) - intersection(2))^2 +...
                (plane.home_point(3) - intersection(3))^2)
    angle = dot(plane.equ(1:3), t(1:3,3)')
    
    sprayPath(plane,0.04,1,'r')

    if checkLevel == 1
        if d < 0.01 & angle > 0.707 %Distance is less than 2cm and angle less than 45deg
            result = 1;
        end
    else
        if d < 0.03 & angle > 0.707 %Distance is less than 2cm and angle less than 45deg
            result = 1;
        end
    end
end
