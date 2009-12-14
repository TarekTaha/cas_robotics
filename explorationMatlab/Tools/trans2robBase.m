function points=trans2robBase(points,rob_base)

%transform with respect to robot base
temp_points=rob_base * [points';ones([1,size(points,1)])];
%normalise again
if any(temp_points(4,:))~=1
    points=temp_points(1:3,:)'./temp_points(4,:);
else
    points=temp_points(1:3,:)';
end