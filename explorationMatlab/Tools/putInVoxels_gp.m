%pass in a set many rows by 3 cols
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
    b=unique(round(a/inc_size))*inc_size;
end
