%% gavin_predict
%
% *Description:* given a set and a model test the model and show results 
%
% *Inputs*
% all_data (3xmany) with angle of incidence, ormalised intensity and rang, 
% all_data_T (1xmany) the correct answer as we specified
% model (SVM model) see: http://www.csie.ntu.edu.tw/~cjlin/libsvm/index.html
%
% *Outputs*
% NONE


function [subset_all_data,subset_all_data_T,subset_all_data_xyz,winner]=gavin_predict(model,all_data,all_data_T,all_data_xyz,ratiotouse)

%so we have to be greater than x*100% sure of all outcome otherwise say unsure
minprob_threshhold=0.7;

if nargin<5 || ratiotouse>1 || ratiotouse<0
    ratiotouse=rand();
end
close all

nummat=max(size(model));

%find out how many of each material points there are
for curr_mat=1:nummat
    num_of_mats(curr_mat)=size(find(all_data_T==curr_mat),1);
end
%choose a number less than the whole set
num_trainingset=round(min(num_of_mats)*ratiotouse);

subset_all_data_T=[];
subset_all_data=[];
subset_all_data_xyz=[];
%for each material select the same number of random points
for curr_mat=1:nummat
    possible_index=find(all_data_T==curr_mat);
    
    %get a random perm of the number of training set
    randindex=randperm(size(possible_index,1));
    randnumofindex=randindex(1:num_trainingset);
    
    %make subset consisting of a random set of each material which is the same size    
    subset_all_data_T=[subset_all_data_T;all_data_T(possible_index(randnumofindex))];
    subset_all_data=[subset_all_data;all_data(possible_index(randnumofindex),:)];
    subset_all_data_xyz=[subset_all_data_xyz;all_data_xyz(possible_index(randnumofindex),:)];
end

%normalise data
subset_all_data=gavin_normalisedata(subset_all_data);

%num of games to be played is sum of first n-1 numbers
num_of_games=((nummat-1)^2+(nummat-1))/2;
numinputs=size(subset_all_data,1);
resulttable=zeros([numinputs,num_of_games]);
probs=zeros([numinputs,num_of_games]);
winner=zeros([numinputs,1]);
randguess=zeros([numinputs,1]);
curr_result=1;
%set up a round robin
for i=1:nummat-1
    for j=i+1:nummat
%         if i~=j
            %will return -1 or 1 depending up if it thinks it is material i
            %or material j

            [resulttable(:,curr_result),nothing,tempprobs]=...
                svmpredict(subset_all_data_T, subset_all_data, model(i,j).val,'-b 1');
             probs(:,curr_result)=max(tempprobs,[],2);

%                         [resulttable(:,curr_result),nothing]=...
%                 svmpredict(randguess, subset_all_data, model(i,j).val);

            %do the change over here
            index_negOne=resulttable(:,curr_result)==-1;
            index_One=resulttable(:,curr_result)==1;
            
            resulttable(index_negOne,curr_result)=i;            
            resulttable(index_One,curr_result)=j;            
%             figure(1);subplot(1,2,1);hist(resulttable(:,1));
%             figure(1);subplot(1,2,2);hist(subset_all_data_T);
%             subset_all_data_T
%             [a,b,c]=svmpredict([tempdata_T_negOne(1:9);tempdata_T_one(1:10)],[tempdata_negOne(1:9,:);tempdata_one(1:10,:)],model(curr_mat_negOne,curr_mat_one).val,'-b 1')
            curr_result=curr_result+1;            
%         end
    end
end


disp_results=false;

for i=1:size(resulttable,1)
    if ~isempty(find(probs(i,:)<minprob_threshhold & probs(i,:)>0,1))
        winner(i)=0;
        if disp_results
            display(['Unsure. Answer:', num2str(resulttable(i,:)),'. Probs: (', num2str(probs(i,:)),'). Correct answer: ', num2str(subset_all_data_T(i))]);
        end
    else
        for j=1:nummat
            winindex=find(resulttable(i,:)==j);
            %wins is now a sum of the certainty, so it we have a certainty of 1
            %then it is a tally of the wins, otherwise it could be as low as
            %0.5, 
            wins(j)=sum(probs(i,winindex));
            %old way
    %         wins(j)=size(find(resulttable(i,:)==j),2);        
        end

        [nothing,winner(i)]=max(wins); 
        %average prob 
        if disp_results
            display(['Pred:', num2str(winner(i)),'. Actual: ', num2str(subset_all_data_T(i)),'.. Results:', num2str(resulttable(i,:)),'. Probs: (', num2str(probs(i,:)),'). Average winner prob is: ',num2str(wins(winner(i))/(nummat-1))]);
        end
    end
end

display(['Of the: ', num2str(size(subset_all_data_T,1)),', I got correct: ',...
    num2str(size(find(subset_all_data_T==winner),1)),...
    ' of ',num2str(size(subset_all_data_T,1)-size(find(winner==0),1)),' (',...
    num2str(size(find(subset_all_data_T==winner),1)/(size(subset_all_data_T,1)-size(find(winner==0),1))*100),...
    '%),and was unsure about ',num2str(size(find(winner==0),1)),...
    ' prob threshold is ',num2str(minprob_threshhold)]);

for curr_mat=1:nummat
    figure(curr_mat)
    thisMaterialTotal=size(find(subset_all_data_T==curr_mat),1);
    %all which are the current material but the answer is not the same as
    %the known answer and we haven't answered unknown
    index=find(subset_all_data_T~=winner & subset_all_data_T==curr_mat & winner~=0);
    misclassifications(curr_mat).val=winner(index);
    hist(misclassifications(curr_mat).val,[1:curr_mat-1,curr_mat+1:nummat])
    xlabel('Incorrect Classifications');
    ylabel(['Incorrect Classifications= ',num2str(size(misclassifications(curr_mat).val,1)),', of ',num2str(thisMaterialTotal)]);
    title(['Material ',num2str(curr_mat),...
        '. Correct: ',num2str(size(find(subset_all_data_T==winner & subset_all_data_T==curr_mat),1)/size(find(subset_all_data_T==curr_mat),1)*100),...
        '%. Unsure: ',num2str(size(find(winner==0 & subset_all_data_T==curr_mat),1)/size(find(subset_all_data_T==curr_mat),1)*100),...
        '%. Mistake: ',num2str(size(find(winner~=0 & winner~=subset_all_data_T & subset_all_data_T==curr_mat),1)/size(find(subset_all_data_T==curr_mat),1)*100),'%'])
%     display('Prese enter to continue');
%     pause
end
