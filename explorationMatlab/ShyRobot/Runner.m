%% Set up UI
colordef black;
i=0;
num_scans_to_take = 50;

%% Get Data
acsor_data =  ACSOR_init();
while(i<num_scans_to_take)
%while(1)
    acsor_data =  GetScanFromACSOR(acsor_data);

%% Prep for next draw
    try
        delete(p)
        delete(m)
    catch
    end

%% 3D Plot - SLOW
%     figure(1); hold on;
%     for x = 1:7
%         for y = 1:100
%             p(x,y) = plot3(x, y, acsor_data.acsor_data_dec(x,y),'.r');
%         end
%     end
%     view(130,30);

%% Mesh
    figure(2);
    x = 1:7;
    y = 1:100;
    m = surf(y, x, acsor_data.acsor_data_dec);
    axis([ 1 100 1 7 0 1000]);
    view(40,30);
    title(['Scan#: ', int2str(i+1), ' of ', int2str(num_scans_to_take)]);
    xlabel('Readings');
    ylabel('Reading Frequency');
    zlabel('Sensor Output Value');
    set(gca,'YTickLabel',{'130kHz','120kHz','110kHz','100kHz','90kHz','80kHz','70kHz'})

    if(i==0)
        waitforbuttonpress();
    end
    
i=i+1;
end

%% Shows a surface for the means of all readings

for j = 1:length(acsor_data.acsor_data_all_readings)
    for i = 1:7
        temp = mean(acsor_data.acsor_data_all_readings{j,1}(i,:));
        acsor_data.all_reading_means(i,j) = temp;
    end
end

waitforbuttonpress();
figure;
x=1:num_scans_to_take;
y=1:7;
s = surf(x,y,acsor_data.all_reading_means);
axis([ 1 num_scans_to_take 1 7 0 1000]);
view(0,30);
xlabel('Scans (each is the average of 100 readings)');
ylabel('Reading Frequency');
zlabel('Sensor Output Value');
set(gca,'YTickLabel',{'130kHz','120kHz','110kHz','100kHz','90kHz','80kHz','70kHz'})
    
%% Clean up
clear x y m s p b i j temp num_scans_to_take;