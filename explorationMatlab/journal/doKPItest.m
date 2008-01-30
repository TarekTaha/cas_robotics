%This function goes through and does all the test that are needed for the
%journal paper

function doKPItest(testnum,tests2do)

try    
close all

%% Check params passed in
if nargin<2
    if nargin<1
        testnum=9;
        display(['testnum being set to ', num2str(testnum)]);    
    end
    tests2do=[1,1,1,1,1,1];
    display(['Doing all test since you didnt say any different!']);
end

if length(tests2do)<6
    a=[a,zeros([1,6-length(a)])];
    display(['You only passed a matrix of size ', length(tests2do), ' - make sure you pass it the size of number of KPIS (6)! Set other test to zero']);
end


%% Setup stuff
% globals
global workspace

%The test cases Need to write this fucking shit down!!
if ~strcmp(pwd,'C:\MATLAB\R2007a\work\explorationMatlab\journal')
    try 
        cd C:\MATLAB\R2007a\work\explorationMatlab\journal
    catch
        display('fucking fix it!!');
    end
end

%run the exGUI at least once
if size(workspace,1)==0
    cd ..
    exGUI
    cd journal
end



%% %%%%%
%|/     /|
%|\PI    |
%%%%%%% ^^^
%\subsubsection{KPI 1 - AXBAM Exploration vs Exhastive Search}
if tests2do(1)==1

    
load(['test',num2str(testnum),'hMesh.mat']);

index=GetImpLevInfo(hMeshdata.v);
index2=find(hMeshdata.v(index,1)>-0.88);
plot3(hMeshdata.v(index(index2),1),hMeshdata.v(index(index2),2),hMeshdata.v(index(index2),3),'r','marker','.','linestyle','none','markersize',0.1);axis equal; grid on

keyboard

















end

%% %%%%%
%|/     ~|
%|\PI   /|\
%%%%%%% ^^
%\subsubsection{KPI 2 - Look Around}
if tests2do(2)==1



end

%% %%%%%
%|/     ~|
%|\PI   ~|
%%%%%%% ~
%\subsubsection{KPI 3 - Termination Conditions}
if tests2do(3)==1


end

%% %%%%%
%|/    | |
%|\PI   ~|
%%%%%%%
%\subsubsection{KPI 4 - Safe Movement}
if tests2do(4)==1

    
    
end

%% %%%%%
%|/     |~~
%|\PI    ~|
%%%%%%% ~~
%\subsubsection{KPI 5 - Map Quality}
if tests2do(5)==1
load(['test',num2str(testnum),'hMesh.mat']);


end


%% %%%%%
%|/     |~
%|\PI   |~|
%%%%%%%  ~
%\subsubsection{KPI 6 - C space opened up}
if tests2do(6)==1
load(['test',num2str(testnum),'hMesh.mat']);



end


%% Overall catch statement
catch
    lasterr
    tempE=lasterror;
    display('question tempE stack for error details if above was not good enough.... you have control');
    keyboard
end
