%% putInVoxels_gp
%
% *Description:* This puts whatever is passed into voxels by dividing by
% the global workspace.inc_size. It will try and use the mex file compiled
% for windows but if fails it will use the matlab way of doing it

%% Function Call
% 
% *Inputs:* 
%
% _a_ (3 x many cartesian matrix)
%
% _incsize_ (double) size of the voxels
%
% *Returns:* 
%
% _b_ (3x many cartesian matrix)

function b=putInVoxels_gp(a,inc_size)

%run the special c function for returning unique put into voxels
% there is a list, followed by a repeated last term, then all zeros
try b=putInVoxels(a,inc_size);
    %find the first repeat (start looking from front)
    firstrepeat=find(b(2:end,1)-b(1:end-1,1)==0 & b(2:end,2)-b(1:end-1,2)==0 & b(2:end,3)-b(1:end-1,3)==0,1);
    %return only unique set
    if ~isempty(firstrepeat)
        b=b(1:firstrepeat,:);
    end    
catch
     b=putInVoxels_matlab(a,inc_size);
end
