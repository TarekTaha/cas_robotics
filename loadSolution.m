function [actions vectors] = loadSolution(filename)
file0 = textread(filename,'%s','delimiter','\n','whitespace','','bufsize',100000);

% remove comments and empty lines (if they exist)
k=0;
for i=1:length(file0)
  comment=strfind(file0{i},'#');
  if ~isempty(comment)
    file0{i}(comment(1):end)=[];
  end
  if ~isempty(file0{i})
    k=k+1;
    file{k}=file0{i};
  end
end
clear file0;
nrLines=length(file);

% dodgy way for getting the number of the states
[s,f,t] = regexp(file{2},'([-\d\.]+)');
[foo,numstates] = size(t);
        
nvectors = nrLines/2;
actions = zeros(1,nvectors);
vectors = zeros(nvectors,numstates);

% process each line
index = 1;
for i=1:nrLines
  if length(file{i})>0
    if(mod(i,2)==0)
        [s,f,t] = regexp(file{i},'([-\d\.]+)');
        [foo,d] = size(t);
        values=zeros(1,d);
        string=file{i};
        for j=1:d
            values(j) = str2double(string(t{j}(1):t{j}(2)));
        end
        vectors(i/2,:) = values;
    else
        actions(index) = sscanf(file{i},'%d');
        index = index + 1;
    end    
end

end