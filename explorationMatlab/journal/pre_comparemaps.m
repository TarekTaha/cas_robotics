close all;

figure(1)
bridgeSection;
hold on;
for i=1:10;
    [u(i),sig(i)]=comparemaps(i);
end;figure; 

bar(u);
figure; 
bar(sig)

figure(1)
cd ..
global r Q
plotdenso(r,Q);
camlight
lighting gouraud
cd journal
axis equal