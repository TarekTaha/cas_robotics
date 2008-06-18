function targetPoints = targetAreaSelection(selectPoints,points)
%Finds the points that are specified within the selected area
%Input: Mesh points; 4 selected boundary points
%Output: The affected points from the mesh map, sorted

%Determine the base plane axis, one with the least variation in a value
variation(1) = max(selectPoints(:,1)) - min(selectPoints(:,1));
variation(2) = max(selectPoints(:,2)) - min(selectPoints(:,2));
variation(3) = max(selectPoints(:,3)) - min(selectPoints(:,3));
[Y I] = min(variation);
switch I
    case 1
        targetPointsTemp = find(points(:,1) < (max(selectPoints(:,1))+0.1) & points(:,1) > (min(selectPoints(:,1))-0.1)...
            & points(:,2) < max(selectPoints(:,2)) & points(:,2) > min(selectPoints(:,2))...
            & points(:,3) < max(selectPoints(:,3)) & points(:,3) > min(selectPoints(:,3))); 
    case 2
        targetPointsTemp = find(points(:,2) < (max(selectPoints(:,2))+0.1) & points(:,2) > (min(selectPoints(:,2))-0.1)...
            & points(:,1) < max(selectPoints(:,1)) & points(:,1) > min(selectPoints(:,1))...
            & points(:,3) < max(selectPoints(:,3)) & points(:,3) > min(selectPoints(:,3))); 
    case 3
        targetPointsTemp = find(points(:,3) < (max(selectPoints(:,3))+0.1) & points(:,3) > (min(selectPoints(:,3))-0.1)...
            & points(:,2) < max(selectPoints(:,2)) & points(:,2) > min(selectPoints(:,2))...
            & points(:,1) < max(selectPoints(:,1)) & points(:,1) > min(selectPoints(:,1))); 
          %targetPoints = find(points(:,3) < (max(selectPoints(:,3))+0.05) & points(:,3) > (min(selectPoints(:,3))+0.05));
end
temp = points(targetPointsTemp,:);
[Y,Index] = sort(temp(:,1),'ascend'); %Sorting in terms of X
for i=1:length(Y)
    targetPoints(i,1) = Y(i);
    targetPoints(i,2) = temp(Index(i),2);
    targetPoints(i,3) = temp(Index(i),3);
end


