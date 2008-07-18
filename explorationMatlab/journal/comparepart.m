%% comparepart
%
% *Description*: You pass in a set of points and this will go through its
% database of parts and determine how well each fits to the data

%% Function Call
%
% *Inputs:* 
%
% _points_ (size?, type?) :input data brought int
%
% *Outputs:* 
% 
% _partsfit_ (size?, type?) :a struct holding part names, their probability
% of existng in environment and transform mats 
%

function [partsfit]=comparepart(points)

%% Variables

%% Input Checks
if nargin<1
  error('need to pass in correct num of vars')
end
if size(points,1)<=1
    error('Need to pass more than 1 points for any comparison');
end

%% Functions
%the parts database
files=dir('*.ply');
if size(files,1)==0
    error('No files in the database to compare to');
end

partsfit.name=files.name;
for i=1:size(files,1)
    partsfit(i).data=plyread([partsfit(i).name,'.ply']);    
end

%% Do comparison
return
