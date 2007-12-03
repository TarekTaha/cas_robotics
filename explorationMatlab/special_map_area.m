function special_map_area()

global RangeData PointData workspace

% load ScanforClassifier-0to-60
% workspace.spec_pnts=[];

[zeroRangeCol,zeroRangeRow]=find(RangeData<20);

%diff te pan direction
diffval=diff(RangeData,1,2);

[diffCol,diffRow]=find(diffval>workspace.inc_size*1000);

% Check that it is not a spike caused by a zero value eg +150 then -150 so
% check the sum over several values
% This also makes sure we don't included specled (varying quickly without
% any value about shadowing)
Istokeep=[];
for i=1:size(diffCol,1)
    if size(diffval,2)>(diffRow(i)+5)
        if abs(sum(diffval(diffCol(i),diffRow(i):diffRow(i)+5)))>workspace.inc_size*1000
            Istokeep=[Istokeep,i];
        end
    else
        if abs(sum(diffval(diffCol(i),diffRow(i):end)))>workspace.inc_size*1000
            Istokeep=[Istokeep,i];
        end    
    end
end

diffCol=diffCol(Istokeep);
diffRow=diffRow(Istokeep);

%special points are the diffCol,diffRow->diffRow+1 values where neither is
%a zero return (these are ignored in this section since it is too hard to
%calculate the areas of interest
for i=1:size(diffCol,1)
    %check if either point is from a zero range point
    if isempty(find(diffCol(i)==zeroRangeCol & zeroRangeRow==diffRow(i),1)) &&...
            isempty(find(diffCol(i)==zeroRangeCol & zeroRangeRow==diffRow(i)+1,1))
        
        radius=sqrt(sum(((PointData(diffCol(i),diffRow(i),:)-PointData(diffCol(i),diffRow(i)+1,:))/2).^2));
        %make sure the radius is not too large
        if radius>2*workspace.inc_size; radius=2*workspace.inc_size; end

        %Determine the mid point between the points with the big
        %differential
        special_point=squeeze((PointData(diffCol(i),diffRow(i),:)+PointData(diffCol(i),diffRow(i)+1,:))/2)';

        %put the point surrounding the special point in areas of interest
        %since they will mean a more complete map
        workspace.spec_pnts=[workspace.spec_pnts;...
            workspace.unknowncoords(...
            find(sqrt((workspace.unknowncoords(:,1)-special_point(1)).^2+...
                      (workspace.unknowncoords(:,2)-special_point(2)).^2+...
                      (workspace.unknowncoords(:,3)-special_point(3)).^2)<radius),:)];   
    end
end

%normalise to the workspace size
workspace.spec_pnts=unique(workspace.spec_pnts,'rows');
workspace.spec_pnts=round(workspace.spec_pnts/workspace.inc_size)*workspace.inc_size;
workspace.spec_pnts=setdiff(workspace.spec_pnts,[workspace.knowncoords;workspace.indexedobsticles],'rows');

% plot3(workspace.spec_pnts(:,1),workspace.spec_pnts(:,2),workspace.spec_pnts(:,3),'k*');
% hold on;
% plot3(PointData(:,:,1),PointData(:,:,2),PointData(:,:,3),'g.')