%% Dynamic Weighting factor using normal fitting

function varargout = tsp_ga_20090915(varargin)

global pose
vidCap = false;
% weighting = 3.4; % A predetermined starting weighting factor for distance
weighting = 10; % A predetermined starting weighting factor for distance
show_progress = true; 
show_results = false;
pfig=[];
display_rate = 4;

if vidCap == true
    movieHandle = avifile('GA_RTA.avi');
end


%TSP_GA  Finds a (near) optimal solution to the Traveling Salesman Problem (TSP)
%   by setting up a Genetic Algorithm (GA) to search for the shortest
%   path (least distance needed to travel to each city exactly once)
%
%   TSP_GA(NUM_CITIES) where NUM_CITIES is an integer representing the number
%   of cities there are (default = 50)
%
%   TSP_GA(CITIES) where CITIES is an Nx2 matrix representing the X,Y
%   coordinates of user specified cities
%
%   TSP_GA(..., OPTIONS) or TSP_GA(OPTIONS) where OPTIONS include one or
%   more of the following in any order:
%     '-NOPLOT'     turns off the plot showing the progress of the GA
%     '-RESULTS'    turns on the plot showing the final results
%   as well as the following parameter pairs:
%     'POPSIZE', VAL  sets the number of citizens in the GA population
%               VAL should be a positive integer (divisible by 4)
%                 -- default = 100
%     'MRATE', VAL  sets the mutation rate for the GA
%               VAL should be a float between 0 and 1, inclusive
%                 -- default = 0.8
%     'NUMITER', VAL  sets the number of iterations (generations) for the GA
%               VAL should be a positive integer
%                 -- default = 500
%
%   Example:
%     % Solves the TSP for 20 random cities using a population size of 60, 
%     % a 75% mutation rate, and 250 GA iterations
%     tsp_ga(20, 'popsize', 60, 'mrate', 0.75, 'numiter', 250);
%
%   Example:
%     % Solves the TSP for 30 random cities without the progress plot
%     [sorted_cities, best_route, distance] = tsp_ga(30, '-noplot');
%
%   Example:
%     % Solves the TSP for 40 random cities using 1000 GA iterations and
%     % plots the results
%     cities = 10*rand(40, 2);
%     [sorted_cities] = tsp_ga(cities, 'numiter', 1000, '-results');
%
%   NOTE: It is possible for TSP_GA to continue where it left off on a
%   previous set of cities by using the sorted city output matrix as an
%   input, as in the following example:
%     cities = 10*rand(60, 2);
%     sorted_cities = tsp_ga(cities, 'numiter', 100);
%     figure; plot(sorted_cities(:, 1), sorted_cities(:, 2), '.-')
%     sorted_cities2 = tsp_ga(sorted_cities);
%     figure; plot(sorted_cities2(:, 1), sorted_cities2(:, 2), '.-')
% 
% Author: Joseph Kirk
% Email: jdkirk630 at gmail dot com
% Release: 1.2
% Release Date: 5/22/07

error(nargchk(0, 9, nargin));
num_cities = 50; num_dims = 3;
cities = 10*rand(num_cities, num_dims);
pop_size = 100; num_iter = 500; mutate_rate = 0.8;


% Process Inputs
cities_flag = 0; option_flag = 0;
for var = varargin
    if option_flag
        if ~isfloat(var{1}), error(['Invalid value for option ' upper(option)]); end
        switch option
            case 'popsize', pop_size = 4*ceil(real(var{1}(1))/4); option_flag = 0;
            case 'mrate', mutate_rate = min(abs(real(var{1}(1))), 1); option_flag = 0;
            case 'numiter', num_iter = round(real(var{1}(1))); option_flag = 0;
            otherwise, error(['Invalid option ' upper(option)])
        end
    elseif ischar(var{1})
        switch lower(var{1})
            case '-noplot', show_progress = 0;
            case '-results', show_results = 1;
            otherwise, option = lower(var{1}); option_flag = 1;
        end
    elseif isfloat(var{1})
        if cities_flag, error('CITIES or NUM_CITIES may be specified, but not both'); end
        if length(var{1}) == 1
            num_cities = round(real(var{1}));
            if num_cities < 2, error('NUM_CITIES must be an integer greater than 1'); end
            cities = 10*rand(num_cities, num_dims); cities_flag = 1;
        else
            cities = real(var{1});
            [num_cities, num_dims] = size(cities); cities_flag = 1;
        end
    else
        error('Invalid input argument.')
    end
end

%% Construct the Joint Matrix - WORK STARTS HERE

temp_pose=zeros([size(pose,2),size(pose(1).Q,2)]);
for i=1:size(cities,1)
    temp_pose(i,:)=pose(i).Q;
end

joint_matx=zeros([size(pose,2),size(pose,2)]);
for i=1:size(pose,2)
    for j=i:size(pose,2)
        %it was originally only the first 5 joints now it is all 6
        joint_matx(i,j)=sqrt((temp_pose(i,1)-temp_pose(j,1))^2)+...
                        sqrt((temp_pose(i,2)-temp_pose(j,2))^2)+...
                        sqrt((temp_pose(i,3)-temp_pose(j,3))^2)+...
                        sqrt((temp_pose(i,4)-temp_pose(j,4))^2)+...
                        sqrt((temp_pose(i,5)-temp_pose(j,5))^2)+...
                        sqrt((temp_pose(i,6)-temp_pose(j,6))^2);
        joint_matx(j,i)=joint_matx(i,j);
    end
end

% Construct the Distance Matrix
mat3d1 = reshape(cities, 1, num_cities, num_dims);
mat3d2 = reshape(cities, num_cities, 1, num_dims);
distance_matx = sqrt(sum((mat3d1(ones(num_cities, 1), :, :) - mat3d2(:, ones(num_cities, 1), :)).^2, 3));


%% Hybrid construction of distance and joint
distNjoint_matx=joint_matx + distance_matx*weighting;

%Create an array to store the fitness of each chromosome produced
%Col1 = distance fitness, Col2 = joint fitness, Col3 = hybrid fitness
fitnessList = zeros(pop_size,3);
weightList = zeros(num_iter,1);

% Initialize Population
pop = zeros(pop_size, num_cities);
% if pop_size<10; error('cant have such a small population size make it more the 11');end
for k=1:min(pop_size,num_cities)
    pop(k, :) = ([k:-1:1,k+1:num_cities]);
end
%if when making up the cities there are less citites then population then
%we have to make up some randome ones to put on the end
if pop_size>num_cities
   for k=num_cities+1:pop_size
       pop(k, :) = randperm(num_cities);
   end 
end
    

% for k = 11:pop_size
%     staysame=k*floor(num_cities/pop_size);
%     if k<pop_size/2        
%         pop(k, :) = [1:staysame,randperm(num_cities-staysame)+staysame];
%     else
%         pop(k, :) = [randperm(num_cities-staysame),num_cities-staysame+1:num_cities];
%     end        
% end

fitness = zeros(1, pop_size);
best_fitness = zeros(1, num_iter); 

% Plot Cities and Distance Matrix in a Figure
pfig=do_plot(show_progress,pfig,cities,distNjoint_matx);


%% MAIN FOR LOOP goes through the required number of iterations
for iter = 1:num_iter
    for p = 1:pop_size
        %this is the joint and weighted distance matrix
        d = distNjoint_matx(pop(p, 1),...
                            pop(p, num_cities));
        %this is just the distance matrix
        distanceFitness = distance_matx(pop(p, 1),...
                                        pop(p, num_cities));
        %this is just the joint matrix                            
        jointFitness = joint_matx(pop(p, 1), pop(p, num_cities));
        
        %add up all the costs for the path of this population
        for city = 2:num_cities
            d = d + distNjoint_matx(pop(p, city-1), pop(p, city));
            distanceFitness = distanceFitness + distance_matx(pop(p, city-1), pop(p, city));
            jointFitness = jointFitness + joint_matx(pop(p, city-1), pop(p, city));
        end
        
        %these are the fitnesses
        fitness(p) = d;
        fitnessList(p,1) = distanceFitness;
        fitnessList(p,2) = jointFitness;
        fitnessList(p,3) = d;        
    end
    
    [best_fitness(iter) index] = min(fitness);
    best_route = pop(index, :);   
    weightList(iter) = weighting;
    %Recalculate the weighting factor and update the values of the dist_matrix for the current population    
%     [u1,sig1]=normfit(fitnessList(:,1)); %Distance
%     [u2,sig2]=normfit(fitnessList(:,2)); %Joint
%     weighting=u2/u1;    
    
    distNjoint_matx = joint_matx + distance_matx*weighting;
    
%% Plots
    if ~mod(iter, display_rate)
        setappdata(pfig,'iter',iter);
        pfig=do_plot(show_progress,pfig,cities,distNjoint_matx,best_fitness,best_route);

        if vidCap
            movieHandle = addframe(movieHandle, getframe(pfig));
        end
    end
    
%     if ~mod(iter, 100) %Early termination base on gradient of change
%         if best_fitness(iter-99)-best_fitness(iter) < 0.01
%             break
%         end
%     end

%% Genetic Algorithm Search    
    pop = genetic_algorithm(pop, fitness, mutate_rate);
end

if vidCap
    close(movieHandle);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if show_progress
    pfig=do_plot(show_progress,pfig,cities,distNjoint_matx,best_fitness,best_route);
end

if show_results
    figure(2)
    imagesc(distNjoint_matx)
    title('Distance Matrix')
    colormap(flipud(gray))
    figure(3)
    plot(best_fitness(1:iter), 'r', 'LineWidth', 2)
    title('Best Fitness')
    xlabel('Generation')
    ylabel('Distance')
    axis([1 max(2, iter) 0 max(best_fitness)*1.1])
    figure(4)
    route = cities([best_route best_route(1)], :);
    plot(route(:, 1), route(:, 2)', 'b.-')
    for c = 1:num_cities
        text(cities(c, 1), cities(c, 2), [' ' num2str(c)], 'Color', 'k', 'FontWeight', 'b')
    end
    title(['Best GA Route (dist = ' num2str(best_fitness(iter)) ')'])
end

%% work out the varargout vals 
[not_used indx] = min(best_route);
best_ga_route = [best_route(indx:num_cities) best_route(1:indx-1)];
if best_ga_route(2) > best_ga_route(num_cities)
    best_ga_route(2:num_cities) = fliplr(best_ga_route(2:num_cities));
end
varargout{1} = cities(best_ga_route, :);
varargout{2} = best_ga_route;
varargout{3} = best_fitness(iter);
varargout{4} = pop;
varargout{5} = fitnessList;
varargout{6} = weightList;


%% --- subfunction: genetic algorithm
function new_pop = genetic_algorithm(pop, fitness, mutate_rate)
[p, n] = size(pop);

% Tournament Selection - Round One
new_pop = zeros(p, n);
ts_r1 = randperm(p);
winners_r1 = zeros(p/2, n);
tmp_fitness = zeros(1, p/2);
for k = 2:2:p
    if fitness(ts_r1(k-1)) > fitness(ts_r1(k))
        winners_r1(k/2, :) = pop(ts_r1(k), :);
        tmp_fitness(k/2) = fitness(ts_r1(k));
    else
        winners_r1(k/2, :) = pop(ts_r1(k-1), :);
        tmp_fitness(k/2) = fitness(ts_r1(k-1));
    end
end

% Tournament Selection - Round Two
ts_r2 = randperm(p/2);
winners = zeros(p/4, n);
for k = 2:2:p/2
    if tmp_fitness(ts_r2(k-1)) > tmp_fitness(ts_r2(k))
        winners(k/2, :) = winners_r1(ts_r2(k), :);
    else
        winners(k/2, :) = winners_r1(ts_r2(k-1), :);
    end
end
new_pop(1:p/4, :) = winners;
new_pop(p/2+1:3*p/4, :) = winners;

% Crossover
crossover = randperm(p/2);
children = zeros(p/4, n);
for k = 2:2:p/2
    parent1 = winners_r1(crossover(k-1), :);
    child = winners_r1(crossover(k), :);
    ndx = ceil(n*sort(rand(1, 2)));
    while ndx(1) == ndx(2)
        ndx = ceil(n*sort(rand(1, 2)));
    end
    tmp = parent1(ndx(1):ndx(2));
    for kk = 1:length(tmp)
        child(child == tmp(kk)) = 0;
    end
    child = [child(1:ndx(1)) tmp child(ndx(1)+1:n)];
    child(child == 0) = [];
    children(k/2, :) = child;
end
new_pop(p/4+1:p/2, :) = children;
new_pop(3*p/4+1:p, :) = children;

% Mutate
mutate = randperm(p/2);
num_mutate = round(mutate_rate*p/2);
for k = 1:num_mutate
    ndx = ceil(n*sort(rand(1, 2)));
    while ndx(1) == ndx(2)
        ndx = ceil(n*sort(rand(1, 2)));
    end
    if rand < 0.75 % swap segment between two cities
        new_pop(p/2+mutate(k), ndx(1):ndx(2)) = ...
            fliplr(new_pop(p/2+mutate(k), ndx(1):ndx(2)));
    else % swap two cities
        new_pop(p/2+mutate(k), [ndx(1) ndx(2)]) = ...
            new_pop(p/2+mutate(k), [ndx(2) ndx(1)]);
    end
end

%% Function Plot
function pfig=do_plot(show_progress,pfig,cities,distNjoint_matx,best_fitness,best_route)
if show_progress
    %don't need to pass in best_fitness
    if isempty(pfig)
        %initialise figure
        pfig = figure;
        subplot(2, 2, 1);        
        plot3(cities(:, 1), cities(:, 2), cities(:,3), 'b.');axis equal        
        num_cities=size(cities,1);
        title([num2str(num_cities) ' Target Points']);       
        
        if num_cities < 75
            for c = 1:num_cities
                text(cities(c, 1), cities(c, 2), [' ' num2str(c)], 'Color', 'k', 'FontWeight', 'b')
            end
        end  
        
        %the overall cities histogram
        subplot(2, 2, 2)
        imagesc(distNjoint_matx)
        title('GA Population Histogram')
        colormap(flipud(gray))

    % need to pass vargin in for all show_progress,pfig
    % this is not the first time this has been run
    else
        %shows the route over the points
        figure(pfig)
        subplot(2, 2, 3)
        route = cities([best_route best_route(1)], :);
        %try and delete previous path
        try 
            theroute_h=getappdata(pfig,'theroute_h');
            delete(theroute_h);
        end        
        theroute_h=plot3(route(:, 1), route(:, 2)', route(:, 3), 'k.-');axis equal
        setappdata(pfig,'theroute_h',theroute_h);
        
        iter=getappdata(pfig,'iter');      
                
        title(['Best GA Route WEIGHTED (fitness = ' num2str(best_fitness(iter)) ')'])
        
        %this shows the value o the best function
        subplot(2, 2, 4)
        plot(best_fitness(1:iter), 'r', 'LineWidth', 2)
        title('Best Fitness')
        xlabel('Generation')
        ylabel('Distance')
        axis([1 max(2, iter) 0 max(best_fitness)*1.1])    
    end 
    
    drawnow
end   
    