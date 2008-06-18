%must only be 3 collums, this will return the sorted rows that are unique.
% for sizes of 100 it is more that twice as fast (takes 40% of the time)
% for sizes of 500 it is 60% of the time
% for sizes of 1000 it is 70% of the time
% for sizes of 10000 it is 85% of the time
% for sizes of 100000 it is 90% of the time
% for sizes of 500000 it is 98% of the time

function b=unique_gp(a)


%sort with 1st colllum having the final and most important index
[ignore,ndx] = sort(a(:,3),'ascend');
[ignore,ind] = sort(a(ndx,2),'ascend');
[ignore,ind] = sort(a(ndx(ind),1),'ascend');
sorted=a(ndx(ind),:);

% %old way
% sorted = sortrows(a);


%find where 1->end-1 is not equal to the next 2-end. Note final row by
%definition is always not equal to others and so is set to 1,1,1
whereNOTequal= [sorted(1:end-1,:)~=sorted(2:end,:);...
                1,1,1];
%return b which is all the sorted rows which are unique            
b=sorted(whereNOTequal(:,1)|whereNOTequal(:,2)|whereNOTequal(:,3),:);
