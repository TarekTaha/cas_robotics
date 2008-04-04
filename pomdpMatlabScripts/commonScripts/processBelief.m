function [maxValue valueFunctions action index] = processBelief(filename,beliefs)
% For Cassandra's output solutions (pomdpSolver output)
% eg: filename = 'solution.alpha';

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

%[actions vectors] = loadSolution(filename);

valueFunctions = zeros(size(actions));

for i=1:length(actions)
    for j=1:length(beliefs)
        valueFunctions(i) = valueFunctions(i) + beliefs(j)*vectors(i,j);
    end
end

[maxValue,index]  = max(valueFunctions);
action = actions(index);
end
