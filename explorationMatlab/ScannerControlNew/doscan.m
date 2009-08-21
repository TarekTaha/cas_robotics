close all;
clear all;
h = actxserver('EyeInHand.ScannerCommand');
global PointData IntensityData AutoGainData
h.Status
h.registerevent(@myhandler);
% h.Type = 'RangeScan';
% h.Mode = 'RangeAndAveragedIntensity';
% h.Width = 240;
    h.Type = 'RangeScan';
    h.Mode = 'RangeAndAveragedIntensity';
    h.Width = 240; 
    h.TiltSpeed=0;
    h.Start(-1);
figure(1)
setappdata(gcf,'opened',1)
subplot(1,2,1)
title('x,y vals')
subplot(1,2,2)
title('intensity')

for i=1:inf;    
%     for j=1:size(PointData,1)
    while size(PointData,1)==0
        pause(0.01);
    end
try delete(a);end
subplot(1,2,1)
    a=plot(PointData(end,:,2),PointData(end,:,3),'r.');
        hold on;
        PrevPointData=[PointData(end,:,2);PointData(end,:,3)];
        try delete(b);end
        b=plot(PrevPointData(1,:),PrevPointData(2,:),'color',[0.5,0,0]);
        axis([-2,2,-2,2]);
subplot(1,2,2)

plot(IntensityData(end,:))
        

        drawnow
        
        if isempty(getappdata(gcf,'opened'))
          h.Stop;
          close all;clear all;
          return;
        end
        
%         pause(0.1);
%     end
    PointData=[];
    IntensityData=[];
end
h.Stop;