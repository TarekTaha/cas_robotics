function poseSelectionCheck(mode)
global r
load planeSet022808
% Using an arbitrary guess of [0 0 0 0 0 0] for all points
if mode == 1
    for i=1:length(planeSet)
        [jointConfig,valid,tempdist]= blasting_posesel(r, planeSet(i).home_point, planeSet(i).equ, [0 0 0 0 0 0], false); 
        if valid == true
            pose(i,:) = jointConfig;
        else
            keyboard
        end
    end
else
    %Randomise order of pose selection, but using previous config as next
    %guess
    randomList = randperm(length(planeSet));
    for i=1:length(randomList)
        if ~exist('jointConfig')
            [jointConfig,valid,tempdist]= blasting_posesel(r, planeSet(randomList(i)).home_point, planeSet(randomList(i)).equ, [0 0 0 0 0 0], false);        
        else
            [jointConfig,valid,tempdist]= blasting_posesel(r, planeSet(randomList(i)).home_point, planeSet(randomList(i)).equ, jointConfig, false);     
        end
        if valid == true
            pose(i,:) = jointConfig;
        else
            keyboard
        end
    end
end
keyboard