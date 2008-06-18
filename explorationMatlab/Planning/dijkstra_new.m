%% dijkstra_new
%
% *Description:*  The dijkstra algorithm implemented in matlab, give links
% and start and end and return a path

%% Function Call
% 
% *Inputs:* 
%
% _n_ (int) the number of nodes in the network;
%
% _s_ source node index;
%
% _d_ destination node index;
%
% *Returns:* 
%
% _path_ the list of nodes in the path from source to destination, 
%
% _totalCost_ the total cost of the path
%
% _farthestNode_ the farthest node to reach for each node after performing the routing

function [path, totalCost] = dijkstra_new(n, netCostMatrix, s, d)


%% Set all the nodes intially to un-visited;
visited(1:n) = 0;

distance(1:n) = inf;    % it stores the shortest distance between each node and the source node;
parent(1:n) = 0;

temp = ones([1,n])*Inf;

distance(s) = 0;

%% Go through each node and calculate all paths
for i = 1:(n-1),   
    for h = 1:n; 
        if visited(h) == 0
            temp(h)=distance(h);
        else
            temp(h)=Inf;
        end; 
    end;
          
     [t, u] = min(temp);    % it starts from node with the shortest distance to the source;
     visited(u) = 1;       % mark it as visited;
     
     tempval=netCostMatrix(u,:)+distance(u);
     tempindex=find(tempval<distance);
     distance(tempindex)=tempval(tempindex);
     parent(tempindex)=u;
end;

%% Determine the best path
path = [];
if parent(d) ~= 0   % if there is a path!
    t = d;
    path = [d];
    while t ~= s
        p = parent(t);
        path = [p path];
        t = p;      
    end;
end;

totalCost = distance(d);

return;