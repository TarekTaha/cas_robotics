function CspaceProb_ofUsage()
global optimise workspace r
qlimits=r.qlim;
leaniancy=optimise.waterPPleaniancy;

workspace.indexedobsticles=[0,0,0];
workspace.knowncoords=workspace.unknowncoords;

matsize=floor((qlimits(1:3,2)-qlimits(1:3,1))./optimise.max_angle_for123'/leaniancy);
stepsize=(qlimits(1:3,2)-qlimits(1:3,1))./(matsize-1);
newQ=zeros([matsize(1)*matsize(2)*matsize(3),6]);
%same as the newQ
Q_usagecount=zeros([matsize(1)*matsize(2)*matsize(3),1]);

for i=1:matsize(1)
    for j=1:matsize(2)
        for k=1:matsize(3)
            newQ((i-1)*matsize(2)*matsize(3)+(j-1)*matsize(3)+k,:)=[qlimits(1,1)+(i-1)*stepsize(1),qlimits(2,1)+(j-1)*stepsize(2),qlimits(3,1)+(k-1)*stepsize(3),0,0,0];
        end
    end
end

for j=1:size(newQ,1)-1
    Q=newQ(j,:);
    newQtemp=newQ(j+1:end,:);
%     newQtemp=newQ([1:j-1,j+1:end],:);
    [pathval] = pathplanner_water(newQtemp,false,false,false);
    for i=1:size(pathval,2)%each        
        if pathval(i).result==1
            toupdate=floor((pathval(i).all_steps(:,1)-qlimits(1,1))/stepsize(1))*matsize(2)*matsize(3)+...
                                 floor((pathval(i).all_steps(:,2)-qlimits(2,1))/stepsize(2))*matsize(3)+...
                                 floor((pathval(i).all_steps(:,3)-qlimits(3,1))/stepsize(3))+1;
                             
            Q_usagecount(toupdate)=Q_usagecount(toupdate)+1;
        end
    end
end
save('Q_usagecount.mat','Q_usagecount','newQ');
figure
subplot(1,2,1);hist(Q_usagecount,1000)
subplot(1,2,2);plot(Q_usagecount)

figure;
scatter3(rad2deg(newQ(find(C>0),1)),rad2deg(newQ(find(C>0),2)),rad2deg(newQ(find(C>0),3)),C(find(C>0))*500,C(find(C>0)),'filled');
xlabel('Joint 1 (deg)');
ylabel('Joint 2 (deg)');
zlabel('Joint 3 (deg)');

figure
plot(Q_usagecount)
xlabel('C-space Nodes')
ylabel('Usage')
set(gca,'Yscale','log')
set(gca,'Xlim',[0,2500])

figure
plot(Q_usagecount/sum(Q_usagecount))
xlabel('C-space Nodes')
ylabel('Probability of Usage')
set(gca,'Xlim',[0,2500])
set(gca,'Yscale','log')
