function loadworkspacestate()
close all;
testnum='8';
testdate='080603';
testdir='.';
% testdir='C:\Documents and Settings\RTA computer\My Documents\My Pictures\';
cd([testdir,'\',testdate]);
for scannum=1:27
    try filename=['AXBAMnC_Test',testnum,'Scan',num2str(scannum),'_workspaceSTATE.mat'];
        tempdata=load(filename);
        sumofclass=tempdata.workspace.ocgrid(:,4)+tempdata.workspace.ocgrid(:,5);
        figure(1)
        hist(sumofclass(find(sumofclass>0)),100)    
        title(['Scan number ',num2str(scannum)]);        
        open(['AXBAMnC_Test',testnum,'Scan',num2str(scannum),'_classNrobot.fig']);
        uiwait(msgbox('press ok to continue'));
    catch
%         break;
    end
end