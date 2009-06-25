%% normalisedata
%
% *Description:* This function will put all_data between 0 and 1
%

function all_data=gavin_normalisedata(all_data)
    all_data=[(all_data(:,1)-min(all_data(:,1)))/(max(all_data(:,1))-min(all_data(:,1))),...
              (all_data(:,2)-min(all_data(:,2)))/(max(all_data(:,2))-min(all_data(:,2))),...
              (all_data(:,3)-min(all_data(:,3)))/(max(all_data(:,3))-min(all_data(:,3)))];
end