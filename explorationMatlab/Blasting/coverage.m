%% coverage
%
% *Description:* This determines the path and coverage for each

function coverage()

close all

%% Variables
%first time you need to load the mesh
load meshNplanes
% plot_planes(plane,0.04);
global r

%define mew
mew=0.04;

Q=[0,0,0,0,0,0];

quickver=false;

findjointconfig=false;

numofitts=10;

%% Get all joint configs
if findjointconfig
    display('Getting joint configurations for all planes');
    tic; warning off;
    for i=1:size(plane,2)
        if i==1; 
            previous_q=Q; 
        else
            if j_sol(i-1).solutionvalid && ~isempty(find(isnan(previous_q),1)); 
                previous_q=j_sol(i-1).qt; 
            else previous_q=Q; 
            end
        end

        try [j_sol(i).qt,j_sol(i).solutionvalid,j_sol(i).dist_val,j_sol(i).targetdist,j_sol(i).used_sol] = ...
            blasting_posesel(r, plane(i).home_point, plane(i).equ, previous_q, quickver);        

            if ~isempty(find(isnan(j_sol(i).qt),1))
                j_sol(i).solutionvalid=0;
            end
    %         j_sol(i)
        catch
            j_sol(i).solutionvalid=0;
        end
    end
    save('j_sol.mat','j_sol');
    toc; warning on;
else %load it from file
    load j_sol.mat
end

allsols=[];for i=1:size(j_sol,2);allsols=[allsols;j_sol(i).solutionvalid];end
allsols_equal_1=find(allsols==1);


%% Go through number of itterations
for currentit=1:numofitts
    jointdiffweight=0;
    coverageweight=0;
    
%% Setting up converage: The mesh points      
    figure(1)    
    cla('reset');
    P0=points;
    subplot(3,1,2)
    plot3(P0(:,1),P0(:,2),P0(:,3),'g.');hold on;axis equal
    P0cover=zeros(size(P0,1),1);
    
%% Generate a random list
    listofplanes=1:size(allsols_equal_1,1);
    randvals=rand(1,size(allsols_equal_1,1));
    [notused,order]=sort(randvals);
    rand_listofplanes=allsols_equal_1(listofplanes(order));

%% Go through each of the list
    previous_q=j_sol(rand_listofplanes(1)).qt;
    
    for i=rand_listofplanes(2:end)'
        jointdiffweight=jointdiffweight+abs(j_sol(i).qt-previous_q);

        %start blast point
        P1=plane(i-1).home_point;

        %End blast point
        P2=plane(i).home_point;

        %distance between P1 and P2
        D12=sqrt((P1(1)-P2(1))^2+(P1(2)-P2(2))^2+(P1(3)-P2(3))^2);
        
        % Arbitary test to see if close enough between planes
    %     if D12<4*mew
            try delete(h1);end
            try delete(h2);end
            h1=plot3(P1(1),P1(2),P1(3),'r*');
            h2=plot3(P2(1),P2(2),P2(3),'b*');
    %     else
            %     display('not close enough');
    %         continue;
    %     end



        %get distance from P1 and P2 to all mesh points
        D02=sqrt((P0(:,1)-P2(1)).^2+(P0(:,2)-P2(2)).^2+(P0(:,3)-P2(3)).^2);
        D10=sqrt((P1(1)-P0(:,1)).^2+(P1(2)-P0(:,2)).^2+(P1(3)-P0(:,3)).^2);

        %fill out mat so same size for cross product
        P2minusP1mat=[ones(size(P0,1),1)*P2(1)-P1(1),...
                      ones(size(P0,1),1)*P2(2)-P1(2),...
                      ones(size(P0,1),1)*P2(3)-P1(3)];

        %get perp distances to surface
        % see this page %
        % http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
        Dh=sqrt(...
                sum(...
                    cross(...
                           P2minusP1mat,...
                           [P1(1)-P0(:,1),P1(2)-P0(:,2),P1(3)-P0(:,3)]...
                           )...
                    .^2 ...
                    ,2)...
                )...
             /...
             sqrt(...
                  sum(...
                      (P2-P1).^2 ...
                      )...
                  );
        %Find points on mesh covered
        %%% mew from line AND 
        %%% where the sum of distance from each home point to the mesh points is less than then the distance between home points + mew
        newlycovered=find(Dh<mew &  D02+D10<D12+mew);
        P0cover(newlycovered)=P0cover(newlycovered)+0.1;
        %make sure it can't have a value greater than 1
%         P0cover(find(P0cover>1))=1;
    
        % update the previous q
        previous_q=j_sol(i).qt;
    end
    
    %display results
    display(['Current Itteration = ',num2str(currentit)]);
    display(['Joint diff weight in rads= ',num2str(jointdiffweight)]);
    figure (1)
    subplot(3,1,1);    
    bar(jointdiffweight)
    title('Absolute Joint Change for each of 6 joints (rads)')
    
    subplot(3,1,2);
    title('Coverage map')
    for j=0.1:0.1:1
        try delete(plothandle(j/0.1)); end
        if j<1
            covered=find(P0cover==j);
        else %when j==1
            covered=find(P0cover>=j);
        end
        if ~isempty(covered)
            plothandle(int32(j/0.1))=plot3(P0(covered,1),P0(covered,2),P0(covered,3),'color',[1-j,1-j,1-j],'marker','.','linestyle','none');
            drawnow
        end
    end
    subplot(3,1,3);    
    hist(P0cover,0:0.1:max(P0cover))
    title('Coverage histogram each 0.1 is covered once');
    uiwait(msgbox('Press to continue'));
end