function runexhastivesearch()
%variables
global r workspace %scan;

qlim=r.qlim;
unknownweight=calunknownweight();

indexed_knowncoords=round(setdiff(workspace.knowncoords(GetImpLevInfo(workspace.knowncoords),:),workspace.indexedobsticles,'rows')/workspace.inc_size);
obsticle_points=workspace.indexedobsticles(GetImpLevInfo(workspace.indexedobsticles),:);

all_possible=round(   workspace.unknowncoords(workspace.lev1unknown   ,:)/workspace.inc_size);
[nothing,index]=setdiff(all_possible,[indexed_knowncoords;obsticle_points],'rows');
unknown_points=workspace.unknowncoords(workspace.lev1unknown(index),:);

exhastive=[];

%the increments between angles
% angle_inc=0.5*pi/180*ones(size(qlim,1),1);
% angle_inc=[5,5,5,45,20].*pi/180;
angle_inc=[20,20,20,45,20].*pi/180;

%calc total num of fields for each joint
total_size=zeros(size(angle_inc,2),1);
for i=1:size(angle_inc,2); total_size(i)=floor((qlim(i,2)-qlim(i,1))/angle_inc(i)); end
display(['The max total possibliities would be',num2str(prod(total_size))]);

for q1_num=1:total_size(1)   
    for q2_num=1:total_size(2)
        for q3_num=1:total_size(3)
            for q4_num=1:total_size(4)
                for q5_num=1:total_size(5)
                    newQ=[q1_num*angle_inc(1) + qlim(1,2),...
                          q2_num*angle_inc(2) + qlim(2,2),...
                          q3_num*angle_inc(3) + qlim(3,2),...
                          q4_num*angle_inc(4) + qlim(4,2),...
                          q5_num*angle_inc(5) + qlim(5,2),...
                          0];
                      
                    [obstacle_result,unknown_result]=check_path_for_col(newQ,obsticle_points,unknown_points);
                    if obstacle_result && unknown_result
                        plot(r,newQ);
                        view(3)
                        tr=fkine(r,newQ');
                        exhastive(end+1).tr=tr;                
                        exhastive(end).scanorigin=round(tr(1:3,4)'/workspace.inc_size)*workspace.inc_size;
                        exhastive(end).Q=newQ;                                
                        exhastive(end).expectedaddinfo=nbv_volume(tr,newQ);
                        exhastive(end).addinfo=getweighted_addinfo(exhastive(end).expectedaddinfo)/unknownweight;
                    end                       
                end
                display('q4');
            end
            display('q3');
        end
        display('q2');
    end
    display('q1');
end
display(['Actual size of exhastive is',num2str(size(exhastive,1))]);
keyboard