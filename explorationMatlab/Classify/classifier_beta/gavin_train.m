%% gavin_train
%
% *Description:* train the SVM classifier
%
% *Inputs*
% all_data (3xmany) with angle of incidence, ormalised intensity and rang, 
% all_data_T (1xmany) the correct answer as we specified
% filename (string) the name of the output model filename (not including _MODEL.mat)
% ratiotouse (0->1 double) this is how much of the set to use in training
%
% *Outputs*
% model (SVM model) see: http://www.csie.ntu.edu.tw/~cjlin/libsvm/index.html

function [model]=gavin_train(all_data,all_data_T,filename,ratiotouse)

if nargin<4 || ratiotouse>1 || ratiotouse<0
    ratiotouse=rand();
    %if there is no filename of the model passed in
    if nargin<3
        filename='latest model';
        warning('Please change the model filename so it wont be overwritten')
    end
end

%find out how many of each material points there are
for curr_mat_negOne=1:max(all_data_T)
    num_of_mats(curr_mat_negOne)=size(find(all_data_T==curr_mat_negOne),1);
end
%choose a number less than the whole set
num_trainingset=round(min(num_of_mats)*ratiotouse);
    
%normalise the data so it is between 0 and 1
all_data=gavin_normalisedata(all_data);

for curr_mat_negOne=1:max(all_data_T)
    indexofthismat_negOne=all_data_T==curr_mat_negOne;
    tempdata_negOne=all_data(indexofthismat_negOne,:);
    tempdata_T_negOne=all_data_T(indexofthismat_negOne,:);    
    
    %get a random perm of the number of training set
    randindex=randperm(size(tempdata_negOne,1));
    randnumofindex=randindex(1:num_trainingset);
    
    %resize the (-1) set of points and the answers
    tempdata_negOne=tempdata_negOne(randnumofindex,:);
    tempdata_T_negOne=tempdata_T_negOne(randnumofindex);
    
    for curr_mat_one=curr_mat_negOne:max(all_data_T)
        if curr_mat_one~=curr_mat_negOne
            indexofthismat_one=all_data_T==curr_mat_one;
            tempdata_one=all_data(indexofthismat_one,:);
            tempdata_T_one=all_data_T(indexofthismat_one,:);                

            %get a random perm of the number of training set
            randindex=randperm(size(tempdata_one,1));
            randnumofindex=randindex(1:num_trainingset);
            
            %resize the (1) set of points and the answers
            tempdata_one=tempdata_one(randnumofindex,:);
            tempdata_T_one=tempdata_T_one(randnumofindex);

            %change the answers to be -1 or 1
            tempdata_T_one(:)=1;
            tempdata_T_negOne(:)=-1;

            %train the curr_mat_negOne vs curr_mat_one model
            try            
                model(curr_mat_negOne,curr_mat_one).val= svmtrain([tempdata_T_negOne;tempdata_T_one],[tempdata_negOne;tempdata_one],'-b 1');                                                     
            catch
                display('some error');
                keyboard;
            end
        end        
    end
end

save([filename,'_MODEL.mat'],'model');
end


    
    