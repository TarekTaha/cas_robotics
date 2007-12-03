function special_map_area()
load ScanforClassifier-0to-60
global RangeData PoseData PointData workspace scan
workspace.spec_pnts=[];

[zeroRangeCol,zeroRangeRow]=find(RangeData<20);
% NewRangeData=RangeData;
% for i=1:size(zeroRangeCol,1)
%     NewRangeData(zeroRangeCol(i),zeroRangeRow(i))=scan.size*1000;
% end

%diff te pan direction
diffval=diff(RangeData,1,2);

[diffCol,diffRow]=find(diffval>100);

% ssect=find(a>130& a<132);
% 
% a(ssect(1))
% b(ssect(1))
% 
Istokeep=[];
for i=1:size(diffCol,1)
%     check if it quickly levels again
    if size(diffval,2)>(diffRow(i)+5)
        if abs(sum(diffval(diffCol(i),diffRow(i):diffRow(i)+5)))>50
            Istokeep=[Istokeep,i];
        end
    else
        if abs(sum(diffval(diffCol(i),diffRow(i):end)))>50
            Istokeep=[Istokeep,i];
        end    
    end
end

diffCol=diffCol(Istokeep);
diffRow=diffRow(Istokeep);

% for i=1:size(a,1);
%     diffval(a(i),b(i)-1:b(i)+1)
%     pause
% end

%special points are the diffCol,diffRow->diffRow+1 values

for i=1:size(diffCol,1)
    radius=sqrt(sum(((PointData(diffCol(i),diffRow(i),:)-PointData(diffCol(i),diffRow(i)+1,:))/2).^2));
    %make sure the radius is not too large
    if radius>2*workspace.inc_size; radius=2*workspace.inc_size; end
    
    %can't use the PointData as was calculated if we substituted
    if isempty(find(diffCol(i)==zeroRangeCol & zeroRangeRow==diffRow(i),1)) &&...
            isempty(find(diffCol(i)==zeroRangeCol & zeroRangeRow==diffRow(i)+1,1))
        special_point=squeeze((PointData(diffCol(i),diffRow(i),:)+PointData(diffCol(i),diffRow(i)+1,:))/2)';
    else
        continue
%         %special point is half between new and old 
%         %pick the min range val point 
%         if NewRangeData(diffCol(i),diffRow(i))<NewRangeData(diffCol(i),diffRow(i)+1)
%            %move shorter one back to the origin by taking away scaner pos
%            shiftedtoorigin=squeeze(PointData(diffCol(i),diffRow(i),:))'-squeeze(PoseData(diffCol(i),1:3,4));
%         else
%            %move shorter one back to the origin by taking away scaner pos
%            shiftedtoorigin=squeeze(PointData(diffCol(i),diffRow(i)+1,:))'-squeeze(PoseData(diffCol(i),1:3,4));
%         end
%         %increase the first by the factor of its size and the radius        
%         scaled=shiftedtoorigin*(NewRangeData(diffCol(i),diffRow(i))+radius*1000)/NewRangeData(diffCol(i),diffRow(i));
%         %add vector to the scanner position vector
%         special_point=scaled+squeeze(PoseData(diffCol(i),1:3,4));
    end

    
    display(['Range vals are ',num2str(RangeData(diffCol(i),diffRow(i))),' to ',num2str(RangeData(diffCol(i),diffRow(i)+1))]);
    display(['Diff value is',num2str(diffval(diffCol(i),diffRow(i)))])
    
    
    temp1=plot3(PointData(diffCol(i),diffRow(i),1),PointData(diffCol(i),diffRow(i),2),PointData(diffCol(i),diffRow(i),3),'r.');
    hold on;
    temp2=plot3(PointData(diffCol(i),diffRow(i)+1,1),PointData(diffCol(i),diffRow(i)+1,2),PointData(diffCol(i),diffRow(i)+1,3),'r.');    
    temp3=plot3(special_point(1),special_point(2),special_point(3),'b*');
    view(3);
    workspace.spec_pnts=[workspace.spec_pnts;...
        workspace.unknowncoords(...
        find(sqrt((workspace.unknowncoords(:,1)-special_point(1)).^2+...
                  (workspace.unknowncoords(:,2)-special_point(2)).^2+...
                  (workspace.unknowncoords(:,3)-special_point(3)).^2)<radius),:)];
              
    plot3(workspace.spec_pnts(:,1),workspace.spec_pnts(:,2),workspace.spec_pnts(:,3),'k*');
%     uiwait(msgbox('ok to continue'));

drawnow
pause(0.5);
try delete(temp1);end
try delete(temp2);end
try delete(temp3);end
      cla('reset')    

    
end

%normalise to the workspace size
workspace.spec_pnts=unique(workspace.spec_pnts,'rows');
workspace.spec_pnts=round(workspace.spec_pnts/workspace.inc_size)*workspace.inc_size;

plot3(workspace.spec_pnts(:,1),workspace.spec_pnts(:,2),workspace.spec_pnts(:,3),'k*');
hold on;
plot3(PointData(:,:,1),PointData(:,:,2),PointData(:,:,3),'g.')