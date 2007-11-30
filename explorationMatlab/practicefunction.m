%practice and generate
function practicefunction()
lookuptable=poselookupGen();
keyboard;


%this will go through all poses at 0.5 deg change and save a T for each one
function lookuptable=poselookupGen()
global r
qlim=r.qlim;

%the increments between angles
% angle_inc=0.5*pi/180*ones(size(qlim,1),1);
% angle_inc=[5,5,5,45,20].*pi/180;
angle_inc=[20,20,20,45,20].*pi/180;

%calc total num of fields for each joint
total_size=zeros(size(angle_inc,2),1);
for i=1:size(angle_inc,2); total_size(i)=floor((qlim(i,2)-qlim(i,1))/angle_inc(i)); end
display(['The total number of elements would be',num2str(prod(total_size))]);
keyboard
T=fkine(r,qlim(:,2));
lookuptable(total_size(1),total_size(2),total_size(3),total_size(4),total_size(5)).val=T;

for q1_num=1:total_size(1)   
    for q2_num=1:total_size(2)
        for q3_num=1:total_size(3)
            for q4_num=1:total_size(4)
                for q5_num=1:total_size(5)
                    q1_val=q1_num*angle_inc(1) + qlim(1,2);
                    q2_val=q2_num*angle_inc(2) + qlim(2,2);
                    q3_val=q3_num*angle_inc(3) + qlim(3,2);
                    q4_val=q4_num*angle_inc(4) + qlim(4,2);
                    q5_val=q5_num*angle_inc(5) + qlim(5,2);
                    
                    T=fkine(r,[q1_val;q2_val;q3_val;q4_val;q5_val;0]);
                    lookuptable(q1_num,q2_num,q3_num,q4_num,q5_num).val=T;
                end
            end
        end
        display('.');
    end
    display('GOGO');
end

keyboard
