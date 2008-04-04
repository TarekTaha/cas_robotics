function policy = zmdpPolicyParse(filename)
%  eg. [maxValue action] = zmdpParse(filename,belief)
% 
% filename: is the name of the file output generated by zmdpSolver usually
% out.policy. 
% belief  : a (1,n) vector with the probability belief of states where n is
% the number of states in the POMDP model. 
display('Reading Policy File');
file = textread(filename,'%s','commentstyle','shell','delimiter','\n','whitespace',' \b\t');

%remove un-necessary info
k=0; maxEnteris=0;
lines = cell(length(file),1);
tic
for i=1:length(file)
    if isempty(file{i})
        continue;
    end
  leftCurlyBracket =strcmp(file{i}(1),'{');
  rightCurlyBracket=strcmp(file{i}(1),'}');
  leftBracket =strcmp((1),'[');
  rightBracket=strcmp(file{i}(1),']');
  policy = strfind(file{i},'policyType');
  planes = strfind(file{i},'planes');
  entries = strfind(file{i},'entries');
  if      ~leftCurlyBracket &&...
          ~rightCurlyBracket &&...
          ~leftBracket &&...
          ~rightBracket &&...
          isempty(policy) &&...
          isempty(planes) &&...
          isempty(entries)

      k=k+1;
      lines{k}=file{i};
       % remove right Arrows
      rightArrow = strfind(lines{k},'=>');
      if ~isempty(rightArrow)
          lines{k}(rightArrow(1):rightArrow(1)+1)=[];
      end     
       % remove right Arrows
      entries = strfind(lines{k},'numEntries');
      if ~isempty(entries)
        x = sscanf(lines{k},'%*s %d');
        if x> maxEnteris
            maxEntries =x;
        end
      end          
       % remove commas
      commas =strfind(lines{k},',');
      l = length(commas);
      if l~= 0
          for j=0:(l-1)
             lines{k}((commas(j+1)-j):(commas(j+1)-j))=[];
          end
      end      
  end
end

k=1;
policy.numPlanes = sscanf(lines{k},'%*s %d');
policy.planeActions = zeros(policy.numPlanes,1);
policy.planeEntries = cell(policy.numPlanes,maxEntries);
k=k+1;

for i=1:policy.numPlanes
    policy.planeActions(i) = sscanf(lines{k},'%*s %d');
    k=k+1;
    [numEntries] = sscanf(lines{k},'%*s %d');
    k=k+1;
    for j=1:numEntries
        x = sscanf(lines{k},'%d %f');
        policy.planeEntries{i,(x(1)+1)} = x(2);
        k=k+1;
    end
    if mod(i,5000) == 0
    %    display(i);
    end
end
display('Policy File Read');
toc
clear lines;
end