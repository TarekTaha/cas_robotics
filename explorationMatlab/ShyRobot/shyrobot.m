function shyrobot()
global Q
if Q(1)~=0
    warning('You should reset the robot so that the first joint is equal to 0');
    Q(1)=0;
end

%% Sensitivity ranges
% Ranges Max
% 1 - Safe moves back to center (+)
sensor_ranges(1)=[1000];move_degs(1)=2;
% 2 - Unsafe (move -1 deg away)
sensor_ranges(2)=[880];move_degs(2)=-1;
% 3 - Unsafe (move -2 deg away)
sensor_ranges(3)=[875];move_degs(3)=-2;
% 4 - Unsafe (move -3 deg away)
sensor_ranges(4)=[875];move_degs(4)=-3;
% 5 - Unsafe (move -4 deg away)
sensor_ranges(5)=[865];move_degs(5)=-4;
% 6 - Unsafe (move -5 deg away)
sensor_ranges(6)=[835];move_degs(6)=-5;
% 7 - Unsafe (move -6 deg away)
sensor_ranges(7)=[820];move_degs(7)=-6;
% 8 - VeryUnsafe (beeps and move -6 deg away)
sensor_ranges(8)=[500];move_degs(8)=-6;




%%
simulateonly=true;
forreal=true;
pausetime=0;

close all;
figure(1);
subplot(1,2,1);axis equal;
camlight
subplot(1,2,2);
axis([0,50,800,900]);

readingnumber=0;
maxiterations=10;

%FOR REAL
if forreal
% Setupscanner
    [acsor_data] =  ACSOR_init();
    while readingnumber<maxiterations %   get actual data
        [acsor_data] =  GetScanFromACSOR(acsor_data);
        av_value=sum(acsor_data.acsor_data_dec(6,:))/size(acsor_data.acsor_data_dec(6,:),2);
        readingnumber=doploting(av_value,readingnumber);
        doshymove(av_value,simulateonly,sensor_ranges,move_degs);
        pause(pausetime)
    end
else %FOR SIMULATION
    load ('ACSOR Data.mat');
    for i=1:size(acsor_data.acsor_data_all_readings,1)    
        lastreading=acsor_data.acsor_data_all_readings{i};
        av_value=sum(lastreading(6,:))/size(lastreading,2);
        readingnumber=doploting(av_value,readingnumber);
        doshymove(av_value,simulateonly,sensor_ranges,move_degs)
        pause(pausetime)
    end
end

%%
%the move function
function doshymove(av_value,simulateonly,sensor_ranges,move_degs)
global Q
%check robot is in correct possition
newQ=Q;

subplot(1,2,1);
%range1
if av_value<sensor_ranges(1) && av_value>=sensor_ranges(2)        
    if Q(1)>=0
        newQ=[0,Q(2:6)];
    else
        newQ=[Q(1)+(move_degs(1)*pi/180),Q(2:6)];
    end
%%range2    
elseif av_value<sensor_ranges(2) && av_value>=sensor_ranges(3)
    newQ=[Q(1)+(move_degs(2)*pi/180),Q(2:6)];
%range3
elseif av_value<sensor_ranges(3) && av_value>=sensor_ranges(4)
    newQ=[Q(1)+(move_degs(3)*pi/180),Q(2:6)];
%range4    
elseif av_value<sensor_ranges(4) && av_value>=sensor_ranges(5)
    newQ=[Q(1)+(move_degs(4)*pi/180),Q(2:6)];
%range5    
elseif av_value<sensor_ranges(5) && av_value>=sensor_ranges(6)
    newQ=[Q(1)+(move_degs(5)*pi/180),Q(2:6)];
%range6    
elseif av_value<sensor_ranges(6) && av_value>=sensor_ranges(7)
    newQ=[Q(1)+(move_degs(6)*pi/180),Q(2:6)];
%range7    
elseif av_value<sensor_ranges(7) && av_value>=sensor_ranges(8)
    newQ=[Q(1)+(move_degs(7)*pi/180),Q(2:6)];
%range8    
elseif av_value<sensor_ranges(8)
    beep
    newQ=[Q(1)+(move_degs(8)*pi/180),Q(2:6)];
end


if newQ(1)<-pi/4
    beep;
    newQ(1)=-pi/4;
end

if simulateonly
    demopath_new(newQ);
    Q=newQ;
else
    demopath_new(newQ);
    use_real_robot_MOVE(newQ);
end

%%
%function for ploting the readings
function readingnumber=doploting(av_value,readingnumber)
readingnumber=readingnumber+1;
subplot(1,2,2);hold on;plot(readingnumber,av_value,'r*');
title(['History of Average Sensor Value; Now at reading ',num2str(readingnumber),' with value ',num2str(av_value)]);xlabel('Reading Number');ylabel('Average Value')
if mod(readingnumber,50)==0
    axis([0,50*(round(readingnumber/50)+1),800,900]);
end
subplot(1,2,1);

    
