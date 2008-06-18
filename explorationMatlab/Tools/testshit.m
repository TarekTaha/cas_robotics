

function testshit()
points=rand([200,3]);
points=[points;points;points;points;points;points;points;points;points;points];
points=[points;points;points;points;points;points;points;points;points;points];
numitts=100;
profile clear; profile on; 
func_to_test(numitts,points)
profile off; profile viewer


function func_to_test(numitts,points)

% set1=putInVoxels_gp(points,0.1); 
% set2=putInVoxels_matlab(points,0.1);
% setdiff(set1,set2,'rows');
for i=1:numitts
%     display('Something or other');
      a=putInVoxels_gp(points,0.1);
end
for i=1:numitts
%     fprintf('Something or other');
    a=putInVoxels_matlab(points,0.1);
end