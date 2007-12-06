function runexhastivesearch()
%variables

global r workspace %scan;
tic

%%%%%%This variable needs to be se
testnum=6;
%the increments between angles
%  angle_inc=0.5*pi/180*ones(size(qlim,1),1);
angle_inc=2*pi/180*ones([1,3]);
% angle_inc=[5,5,5,45,20].*pi/180;
% angle_inc=[20,20,20,45,20].*pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%

%this is so we can load the test workspace results
cd journal' results'\
load(['test',num2str(testnum),'workspace.mat']);
cd ..

qlim=r.qlim.*0.98;
unknownweight=calunknownweight();

indexed_knowncoords=round(setdiff(workspace.knowncoords(GetImpLevInfo(workspace.knowncoords),:),workspace.indexedobsticles,'rows')/workspace.inc_size);
obsticle_points=workspace.indexedobsticles(GetImpLevInfo(workspace.indexedobsticles),:);

all_possible=round(   workspace.unknowncoords(workspace.lev1unknown   ,:)/workspace.inc_size);
[nothing,index]=setdiff(all_possible,[indexed_knowncoords;obsticle_points],'rows');
unknown_points=workspace.unknowncoords(workspace.lev1unknown(index),:);





% %calc total num of fields for each joint
total_size=zeros(size(angle_inc,2),1);
for i=1:size(angle_inc,2); total_size(i)=floor((qlim(i,2)-qlim(i,1))/angle_inc(i)); end
display(['The max total possibliities would be',num2str(prod(total_size))]);

% exhastive=[];
% for q1_num=1:total_size(1)   
%     for q2_num=1:total_size(2)
%         for q3_num=1:total_size(3)
%             for q4_num=1:total_size(4)
%                 for q5_num=1:total_size(5)
%                     newQ=[q1_num*angle_inc(1) + qlim(1,2),...
%                           q2_num*angle_inc(2) + qlim(2,2),...
%                           q3_num*angle_inc(3) + qlim(3,2),...
%                           q4_num*angle_inc(4) + qlim(4,2),...
%                           q5_num*angle_inc(5) + qlim(5,2),...
%                           0];
%                       
%                     [obstacle_result,unknown_result]=check_path_for_col(newQ,obsticle_points,unknown_points);
%                     if obstacle_result && unknown_result
%                         plot(r,newQ);
%                         view(3)
%                         tr=fkine(r,newQ');
%                         exhastive(end+1).tr=tr;                
%                         exhastive(end).scanorigin=round(tr(1:3,4)'/workspace.inc_size)*workspace.inc_size;
%                         exhastive(end).Q=newQ;                                
%                         exhastive(end).expectedaddinfo=nbv_volume(tr,newQ);
%                         exhastive(end).addinfo=getweighted_addinfo(exhastive(end).expectedaddinfo)/unknownweight;
%                     end                       
%                 end
%                 display('q4');
%             end
%             display('q3');
%         end
%         display('q2');
%     end
%     display('q1');
% end
% display(['Actual size of exhastive is',num2str(size(exhastive,1))]);
% keyboard

all_data=zeros([total_size(1)*total_size(2)*total_size(3),5]);
count=1;

for q1_num=1:total_size(1)   
    for q2_num=1:total_size(2)
        for q3_num=1:total_size(3)
            newQ=[q1_num*angle_inc(1) + qlim(1,1),...
                  q2_num*angle_inc(2) + qlim(2,1),...
                  q3_num*angle_inc(3) + qlim(3,1),...
                  0,0,0];
                      
            [obstacle_result,unknown_result]=check_path_for_col(newQ,obsticle_points,unknown_points);
            all_data(count,:)=[newQ(1:3),obstacle_result,unknown_result];
            count=count+1;
        end
    end                  
end

    try 
    save('config_space.mat','all_data');
    cd journal' results'\
    movefile('../config_space.mat',['test',num2str(testnum),'config_space.mat']);
    cd ..
catch
    display('Some error - user has control');
    keyboard;
end

%ok =1, not ok =0;
obstacles=find(all_data(:,4)==0);
unknown=find(all_data(:,4)==1 & all_data(:,5)==0);
free=find(all_data(:,4)==1 & all_data(:,5)==1);

figure(2)

plot3(all_data(obstacles,1),all_data(obstacles,2),all_data(obstacles,3),'b','marker','.','markersize',angle_inc(1)*180/pi,'linestyle','none');
% plot3(all_data(obstacles,1),all_data(obstacles,2),all_data(obstacles,3),'b','linestyle','none');
hold on;
plot3(all_data(unknown,1),all_data(unknown,2),all_data(unknown,3),'y','marker','.','markersize',angle_inc(1)*180/pi,'linestyle','none')
% plot3(all_data(unknown,1),all_data(unknown,2),all_data(unknown,3),'y','linestyle','none')
plot3(all_data(free,1),all_data(free,2),all_data(free,3),'g','marker','*','markersize',angle_inc(1)*180/pi,'linestyle','none')
title(['Test',num2str(testnum),'Discretised Configuration Space']);
xlabel('Joint 1 (rads)')
ylabel('Joint 2 (rads)')
zlabel('Joint 3 (rads)')
grid on;

display(['Test #',num2str(testnum),': With inc between steps of joints 1->3 of: [' num2str(angle_inc*180/pi),'] degrees, and sample space of: ', num2str(size(all_data,1)),' voxels']);
display(['Percentage of config space free is approx: ' num2str(size(free,1)/size(all_data,1)*100),'%']);
display(['Percentage of config space unknown is approx: ' num2str(size(unknown,1)/size(all_data,1)*100),'%']);
display(['Percentage of config space obstacled is approx: ' num2str(size(obstacles,1)/size(all_data,1)*100),'%']);

toc

