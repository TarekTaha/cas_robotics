% function to order the SLD given a group of planes normal plane structure
% and all_poses in the form of a matrix of poses
function [planeSet, all_poses]=orderSLDs(planeSet, all_poses)


global pose

numiter=100;
popsize=100;

pose=[];cities=[];

cities(1,:)=planeSet(1).home_point;
pose(1).Q = all_poses(1,:);

validindex=1;
for i=2:size(planeSet,2)   
    if all_poses(i,8)
        cities(validindex,:)=planeSet(validindex).home_point;
        pose(validindex).Q = all_poses(validindex,1:7);
        validindex=validindex+1;
    end
end

%% Run the TSP on the cities
[sorted_cities, best_route] = tsp_ga_20090915(cities, 'popsize', popsize, 'mrate', 0.75, 'numiter', numiter);

    
planeSet=planeSet(best_route);
all_poses=all_poses(best_route,:);
