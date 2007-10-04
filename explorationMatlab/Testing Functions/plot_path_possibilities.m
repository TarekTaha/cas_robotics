%this function will display look the results from demo path that checked
%the joints being in states from max to min and then the current state will
%be a read line through the maze of configuration space, with blue steps
%where there are destinations
load allFFresults_83paths2middleQ.mat
load pathdata.mat
close all;

jointlimits=r.qlim;

pathstarts_1=[];
pathstarts_2=[];
pathstarts_3=[];
pathsteps=1;
for j=1:size(pathdata,2)   
    pathstarts_1=[pathstarts_1;pathsteps,[pathdata(j).all_steps(1,1)]];
    pathstarts_2=[pathstarts_2;pathsteps,[pathdata(j).all_steps(1,2)]];
    pathstarts_3=[pathstarts_3;pathsteps,[pathdata(j).all_steps(1,3)]];
    pathsteps=pathsteps+size(pathdata(j).all_steps,1);
end



todo=size(allFFresults.joint1_data,2);
figure;
newdata=[];
for i=1:size(allFFresults.joint1_data,2);newdata=[newdata,find(allFFresults.joint1_data(:,i)==0.5,1)];if i==todo; break; end; end
subplot(3,1,1);imshow(allFFresults.joint1_data(:,1:todo)); hold on; plot(newdata,'r');plot(pathstarts_1(:,1),newdata(pathstarts_1(:,1)),'b*');

newdata=[];
for i=1:size(allFFresults.joint2_data,2);newdata=[newdata,find(allFFresults.joint2_data(:,i)==0.5,1)];if i==todo; break; end;end
subplot(3,1,2);imshow(allFFresults.joint2_data(:,1:todo));hold on; plot(newdata,'r');plot(pathstarts_2(:,1),newdata(pathstarts_2(:,1)),'b*');

newdata=[];
for i=1:size(allFFresults.joint3_data,2);newdata=[newdata,find(allFFresults.joint3_data(:,i)==0.5,1)];if i==todo; break; end;end
subplot(3,1,3);imshow(allFFresults.joint3_data(:,1:todo));hold on; plot(newdata,'r');plot(pathstarts_3(:,1),newdata(pathstarts_3(:,1)),'b*');
