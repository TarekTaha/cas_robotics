%this is a function to test the streamOnto_mine_manystarts pose selection
%function and then save the results of the distance to the target and if it
%is a valid solution. If there is a collsion or out of joint limits then it
%is not counted as a valid solution
all_valid=[];
all_joints=[];
all_used_sol=[];
all_dist_val=[];

for i=1:size(a,1);
    [Q,valid,used_sol,dist_val]=streamOnto_mine_manystarts(r,a(i,:),2*rand(1,3)-1,Q);
    pause(0.05);
    all_valid=[all_valid;valid];
    all_joints=[all_joints;Q];
    all_used_sol=[all_used_sol;used_sol];
    all_dist_val=[all_dist_val;dist_val];
    display(strcat('Valid=',num2str(valid),'; Usedsol=',num2str(used_sol),'; Dist=',num2str(dist_val)));
end

data.all_valid=all_valid;
data.all_joints=all_joints;
data.all_used_sol=all_used_sol;
data.all_dist_val=all_dist_val;


save('poseselect_data.mat','data')