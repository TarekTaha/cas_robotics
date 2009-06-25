%% gavin_classifiy
%
% *Description:* use the SVn classifier
%

function [model]=gavin_train(all_data,all_data_T,filename)

%if there is no filename of the model passed in
if nargin<3
    filename='latest model';
    warning('Please change the model filename so it wont be overwritten')
end

%normalise the data so it is between 0 and 1
all_data=gavin_normalisedata(all_data);

for curr_mat_negOne=1:max(all_data_T)
    indexofthismat_negOne=all_data_T==curr_mat_negOne;
    tempdata_negOne=all_data(indexofthismat_negOne,:);
    tempdata_T_negOne=all_data_T(indexofthismat_negOne,:);    
    for curr_mat_one=curr_mat_negOne:max(all_data_T)
        if curr_mat_one~=curr_mat_negOne
            indexofthismat_one=all_data_T==curr_mat_one;
            tempdata_one=all_data(indexofthismat_one,:);
            tempdata_T_one=all_data_T(indexofthismat_one,:);                
            
            tempdata_T_one(:)=1;
            tempdata_T_negOne(:)=-1;

            model(curr_mat_negOne,curr_mat_one).val= svmtrain([tempdata_T_negOne;tempdata_T_one],[tempdata_negOne;tempdata_one],'-b 1');                                                     
        end        
    end
end

save([filename,'_MODEL.mat'],'model');
end


    
    