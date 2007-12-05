function comparemaps()

%load the mesh map object
testnum=1;

load(['test',num2str(testnum),'hmesh.mat');

hmesh.verts


%make some planes that describe the surface, by having points which are the
%surface and running the surface making algorithm
xpoints=-2:0.01:2';
ypointstop=-1:0.01:0';
ypointweb=-0.9;
ypointflange=-1:0.01:-0.8';
Zpointstop=0.6;
Zpointsweb=0:0.01:0.6';
Zpointsflange=0;

for part=1:3
for x=xpoints
    for y=ypointstop
env_points=[env_points;...
    xpoints,Zpointstop,ypointstop];

surface_making_simple()
