function [s f t obs] = learnObservationModel(traningFilename)

file = textread(traningFilename,'%s','delimiter','\n','whitespace',' \b\t','bufsize',100000);
k=0;
fd = fopen('fileout.txt','w');
%ignore comments
for i=1:length(file)
  comment=strfind(file{i},'#');
  if ~isempty(comment)
    file{i}(comment(1):end)=[];
  end
  if ~isempty(file{i})
    k=k+1;
    fileTemp{k}=file{i};
  end
end
clear file;

% read tasks (sequence of observations and locations)
numObs=0;
numendLocations = 0;
for i=1:length(fileTemp)  
  pat='(\d)+\s*(\S+)\s*';
  [s,f,t]=regexp(fileTemp{i},pat); 
  last = length(t);
%   % Find the destination index from the set of given destinations
%   dest = str2num(fileTemp{i}(t{last}(1,1):t{last}(1,2)));
%   indx = strfind(pomdpModel.destinations,sprintf('s%dd',dest));
%   for j=1:length(indx)
%       if ~isempty(indx{j})
%         destIndx = j;
%       end 
%   end
%   if ~exist('destIndx')
%       error('Undefined Destination !!!');
%   end
%   % build the observation set from this task
   for j=1:length(t)
       numObs = numObs + 1;
       obs{numObs}.obs   = fileTemp{i}(t{j}(2,1):t{j}(2,2));
       obs{numObs}.pos   = str2num(fileTemp{i}(t{j}(1,1):t{j}(1,2)));
       fprintf(fd,'%d %s ',obs{numObs}.pos,obs{numObs}.obs);
   end
   fprintf(fd,'\n');
   numObs=0;
   for j=1:length(t)
       numObs = numObs + 1;
       obs{numObs}.obs   = fileTemp{i}(t{j}(2,1):t{j}(2,2));
       obs{numObs}.pos   = str2num(fileTemp{i}(t{j}(1,1):t{j}(1,2)));
       fprintf(fd,'%d %s ',obs{numObs}.pos,'Nothing');
   end
   fprintf(fd,'\n');
end

clear fileTemp;
end