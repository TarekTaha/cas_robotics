function confidenceofpoints()
load GavData

mew=0.10;

points=zeros([size(PointData,1)*size(PointData,2),3]);
laser_pos=zeros([size(PointData,1)*size(PointData,2),3]);
for i=1:size(PointData,1);
    points((i-1)*size(PointData,2)+1 : i*size(PointData,2) , :)=...
        [[PointData(i,:,1)]',[PointData(i,:,2)]',[PointData(i,:,3)]'];    
    laser_pos((i-1)*size(PointData,2)+1 : i*size(PointData,2) , :)=...
        ones([size(PointData,2),1])*[PoseData(i,1,4),PoseData(i,2,4),PoseData(i,3,4)];    
end

surface_making_simple(points,mew)
global plane

pointsPlane=zeros([size(PointData,1)*size(PointData,2),1]);
smallest_disttoplan=inf*ones([size(PointData,1)*size(PointData,2),1]);
for i=1:size(plane,2)
    disttoplane=sqrt((plane(i).home_point(1)-points(:,1)).^2+(plane(i).home_point(2)-points(:,2)).^2+(plane(i).home_point(2)-points(:,3)).^2);
    pointsPlane(disttoplane<smallest_disttoplan)=i;   
    smallest_disttoplan(disttoplane<smallest_disttoplan)=disttoplane(disttoplane<smallest_disttoplan);
end



%rays
vectorvals=(points-laser_pos);
vectorvals=vectorvals/norm(vectorvals);
angleofincidence=zeros([size(PointData,1)*size(PointData,2),1]);
for i=1:size(pointsPlane,1)
    normvec=[plane(pointsPlane(i)).equ(1),plane(pointsPlane(i)).equ(2),plane(pointsPlane(i)).equ(3)];    
    angleofincidence(i)=acos(dot(normvec,vectorvals(i,:)));
end

imshow(angleofincidence/max(max(angleofincidence)))
