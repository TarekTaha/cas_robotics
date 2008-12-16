%% get_unknown_identification
function [index,UNclassifiedvoxels]=get_unknown_identification()

global workspace alldirectedpoints

doNEWway=true;
classifyProbThreshhold=workspace.classifyProbThreshhold;
mew=workspace.classifyMew;
class_cubesize=workspace.class_cubesize;

if doNEWway
%% New way

    UNclassifiedvoxels=find(max(workspace.probofmaterial(:,4:end)')<classifyProbThreshhold)';
    %make new surfaces out of unclassified voxels
    surface_making_simple(workspace.probofmaterial(UNclassifiedvoxels,1:3)*class_cubesize,mew)
    
else
    
%% Old way
    %determine the unknown places and the know metal or wood
    sumofclass=workspace.ocgrid(:,4)+workspace.ocgrid(:,5);
    warning('off','MATLAB:divideByZero')
    % a voxel is unknown if 1&2 OR 3&4
    % 1) the sum of actual classifications is less than minclassifications 
    % and 2) the unknown classifications is still less than minclassifications (else its too hard to classify(like corners)
    % 3) Wood classifgications are not significantly more than metal
    % 4) Methal classifications are not significantly more than wood

    try UNclassifiedvoxels=find((sumofclass<workspace.minclassifications &...
                                workspace.ocgrid(:,6)<workspace.minclassifications)...
                            | ...
                         (workspace.ocgrid(:,4)./workspace.ocgrid(:,5)<=workspace.classfierthreshhold &...
                          workspace.ocgrid(:,5)./workspace.ocgrid(:,4)<=workspace.classfierthreshhold));end
    warning('on','MATLAB:divideByZero')
    
    %make new surfaces out of unclassified voxels
    surface_making_simple(workspace.ocgrid(UNclassifiedvoxels,1:3)*class_cubesize,mew)

end



%% Determine the uncertain region surfaces

global plane
if size(plane,2)<1
    display('There are no more points to look at - returning');
    index=[];
    return
end

% Find the planes covering most unknown points
% these are desirable places to look
sizemat=zeros([length(plane),1]);
for i=1:length(plane)
    [level1,level2]=GetImpLevInfo(plane(i).home_point);
    if ~isempty(level2)
        if ~isempty(alldirectedpoints)
            if isempty(find(sqrt((plane(i).home_point(1)-alldirectedpoints(:,1)).^2+(plane(i).home_point(2)-alldirectedpoints(:,2)).^2+(plane(i).home_point(3)-alldirectedpoints(:,3)).^2)<2*mew,1))
                sizemat(i)=size(plane(i).points,1);
            end
        else
            sizemat(i)=size(plane(i).points,1);
        end
    end
end

% order these
[nothing,index]=sort(sizemat,'descend');
indextoblast=1;  