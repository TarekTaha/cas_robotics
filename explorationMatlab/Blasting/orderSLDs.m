% function to order the SLD given a group of planes normal plane structure
% and all_poses in the form of a matrix of poses
function [planeSet, all_poses,geo_dist,joint_dist]=orderSLDs(planeSet, all_poses)


global pose

numiter=100;
popsize=100;

pose=[];cities=[];geo_dist=[];joint_dist=[];

% cities(1,:)=planeSet(1).home_point;
% pose(1).Q = all_poses(1,:);

validindex=0;
for i=1:size(planeSet,2)   
    if size(all_poses,2)<8 || all_poses(i,8)
        validindex=validindex+1;
        cities(validindex,:)=planeSet(validindex).home_point;
        pose(validindex).Q = all_poses(validindex,1:7);        
    end
end

%% Run the TSP on the cities
[sorted_cities, best_route] = tsp_ga_20090915(cities, 'popsize', popsize, 'mrate', 0.75, 'numiter', numiter);


% MUST DO THIS REORDERING
%if all poses are valid (EASY CASE)
if size(all_poses,2)<8 || isempty(find(all_poses(i,8)==0,1))    
    planeSet=planeSet(best_route);
    all_poses=all_poses(best_route,:);
else %need to reconstruct planeSet and all_poses (HARD CASE)
    all_poses=[];    planeSet=[];   curr_index=0;
    for i=best_route'
        curr_index=curr_index+1;
        all_poses(curr_index,:)=[pose(i).Q,1];
        planeSet(curr_index,:)=cities(i,:);
    end
end


%% determine the 
if nargout>2
    geo_dist= sum(dist2pts(sorted_cities(1:end-1,:),sorted_cities(2:end,:)));
    joint_dist=sum(abs(all_poses(1:end-1,1:6)-all_poses(2:end,1:6)),1);
end
        
