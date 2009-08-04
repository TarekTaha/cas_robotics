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

for i=1:inf;    
%     for j=1:size(PointData,1)
    while size(PointData,1)==0
        pause(0.01);
    end
try delete(a);end
figure(1)
    a=plot(PointData(end,:,2),PointData(end,:,3),'r.');
        hold on;
        PrevPointData=[PointData(end,:,2);PointData(end,:,3)];
        try delete(b);end
        b=plot(PrevPointData(1,:),PrevPointData(2,:),'color',[0.5,0,0]);
        axis([-2,2,-2,2]);
figure(2)

plot(IntensityData(end,:))
        

        drawnow
        
        
%         pause(0.1);
%     end
    PointData=[];
    IntensityData=[];
end
h.Stop;