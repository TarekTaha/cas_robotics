function gavin_predict(model,all_data,all_data_T)

close all

%rand num and rand perm of solutions 
randindex=randperm(size(all_data,1));
randnumofindex=1:round(rand*1000);
subset_all_data_T=all_data_T(randindex(randnumofindex));
subset_all_data=all_data(randindex(randnumofindex),:);
%normalise data
subset_all_data=gavin_normalisedata(subset_all_data);

nummat=max(size(model));
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



for i=1:size(resulttable,1)
    for j=1:nummat
        wins(j)=size(find(resulttable(i,:)==j),2);
    end
    [nothing,winner(i)]=max(wins);
end

display(['Got correct: ',...
    num2str(size(find(subset_all_data_T==winner),1)),...
    ' of ',num2str(size(subset_all_data_T,1)),' (',...
    num2str(size(find(subset_all_data_T==winner),1)/size(subset_all_data_T,1)*100),'%)']);

for curr_mat=1:nummat
    figure(curr_mat)
    thisMaterialTotal=size(find(subset_all_data_T==curr_mat),1);
    index=find(subset_all_data_T~=winner & subset_all_data_T==curr_mat);
    misclassifications(curr_mat).val=winner(index);
    hist(misclassifications(curr_mat).val,[1:curr_mat-1,curr_mat+1:nummat])
    xlabel('Incorrect Classifications');
    ylabel(['Incorrect Classifications= ',num2str(size(misclassifications(curr_mat).val,1)),', of ',num2str(thisMaterialTotal),', with all tested= ',num2str(size(randnumofindex,2)),'']);
    title(['Should be material ',num2str(curr_mat)])
    pause
end
