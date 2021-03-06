function policy = zmdpPolicyParse(filename)
% eg. [maxValue action] = zmdpParse(filename,belief)
% -Input:
%   filename = is the name of the file output generated by zmdpSolver usually
%   out.policy. 
% -Output:
%   policy = the parsed policy, a structure with the number of hyper planes
%   with reward values associated with each state and an output action.

% ************************************************************************\
% * Copyright (C) 2007 - 2008 by:                                         *
% *    Tarek Taha, Jaime Valls Miro, Gamini Dissanayake                   *
% *     CAS-UTS  {t.taha,j.vallsmiro,g.dissanayake}@cas.edu.au            *
% *                                                                       *
% *                                                                       *
% * This program is free software; you can redistribute it and/or modify  *
% * it under the terms of the GNU General Public License as published by  *
% * the Free Software Foundation; either version 2 of the License, or     *
% * (at your option) any later version.                                   *
% *                                                                       *
% * This program is distributed in the hope that it will be useful,       *
% * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
% * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
% * GNU General Public License for more details.                          *
% *                                                                       *
% * You should have received a copy of the GNU General Public License     *
% * along with this program; if not, write to the                         *
% * Free Software Foundation, Inc.,                                       *
% * 51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
% ************************************************************************/

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
maxDim =0;
for i=1:policy.numPlanes
    policy.planeActions(i) = sscanf(lines{k},'%*s %d');
    k=k+1;
    [numEntries] = sscanf(lines{k},'%*s %d');
    k=k+1;
    for j=1:numEntries
        x = sscanf(lines{k},'%d %f');
        policy.planeEntries{i,(x(1)+1)} = x(2);
        if (x(1)+1) > maxDim
            maxDim = (x(1)+1);
        end
        k=k+1;
    end
    if mod(i,5000) == 0
    %    display(i);
    end
end

% check if this vector is missing the relevant information
% then give it a large negative number to insure that this 
% vector will not be picked up later on
for i=1:policy.numPlanes
    for j=1:maxDim
        if  isempty(policy.planeEntries{i,j})
            policy.planeEntries{i,j} = -1000000;
        end
    end
end


display('Policy File Read');
toc
clear lines;
end