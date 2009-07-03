%% gavin_prob_classifier
%
% *Description:* This uses the mean and std to classify
%
% *Inputs*
% all_data (3xmany) with angle of incidence, ormalised intensity and rang, 
% all_data_T (1xmany) the correct answer as we specified
% all_data_xyz (3xmany) xyz points which correspond to the data
%
% *Outputs*
% winner (int 0->nummats) 0 is unzure then 1-> num mats for that material

function [winner,voxels]=gavin_prob_classifier(all_data,all_data_T,all_data_xyz)

angle_inc=10;
startangle=5;
stopangle=85;
range_inc=0.1;
startrange=0+range_inc/2;
stoprange=4-range_inc/2;
voxelsize=0.05;

numOfMats=max(all_data_T);

%allocate maximum memory
lookuptable=zeros([max(all_data_T)*...
    size(startrange:range_inc:stoprange,2)*...
    size(startangle:angle_inc:stopangle,2),5]);

count=1;

%go through each material
for material_num=1:max(all_data_T)
    angle=all_data(:,1);
    intensity=all_data(:,2);
    range=all_data(:,3);

    %go through each range increment
    for range_temp=startrange:range_inc:stoprange       
        index=find(range>=range_temp-range_inc/2&...
                   range<range_temp+range_inc/2&...
                   all_data_T==material_num);
        if size(index,1)>0
            %go through each angle increment
            for angle_temp=startangle:angle_inc:stopangle;
                index2=find(angle(index)>=angle_temp-angle_inc/2&...
                            angle(index)< angle_temp+angle_inc/2);
                if size(index2,1)>0
                    [muhat,sigmahat] = normfit(intensity(index(index2)));                   
                    %also include the number of values in here so this can
                    %weight the probability
                    lookuptable(count,:)=[angle_temp,range_temp,muhat,sigmahat,material_num];
                    
                    count=count+1;
                    
%                     if (85>=angle_temp-angle_inc/2 && 85< angle_temp+angle_inc/2 && 0.65 >=range_temp-range_inc/2 && 0.65<range_temp+range_inc/2)
%                     figure(5);
% %                     cla('reset')                    
% subplot(1,3,material_num);
%                      plot(0:0.01:1,normpdf(0:0.01:1,muhat,sigmahat),'g');hold on;
%                       hist(intensity(index(index2)))
%                     title(['angle_temp:',num2str(angle_temp),', range_temp:',num2str(range_temp),' material_num:',num2str(material_num)]);
%                     drawnow;
%                     pause(0.5)                        
%                     end
                end 
            end
        end        
    end
end


lookuptable=lookuptable(1:count-1,:);

%we have defined 3 possible colours here        
%1) rusted metal 2) wood metal 3) shiny metal
currentcolor=['r','g','b'];
figure(4);hold on;
for material_num=1:numOfMats
    index=find(lookuptable(:,5)==material_num);
    plot3(lookuptable(index,1),lookuptable(index,3),lookuptable(index,2),'color',currentcolor(material_num),'marker','*','markersize',8,'linestyle','none');hold on;
    plot3(lookuptable(index,1),lookuptable(index,3)+lookuptable(index,4),lookuptable(index,2),'color',currentcolor(material_num),'marker','.','markersize',4,'linestyle','none');hold on;
    plot3(lookuptable(index,1),lookuptable(index,3)-lookuptable(index,4),lookuptable(index,2),'color',currentcolor(material_num),'marker','.','markersize',4,'linestyle','none');hold on;
end






%define voxels and initialise a uniform distribution across all materials
% all_data_xyz;
voxels=round(putinVoxels_gp(all_data_xyz,voxelsize)/voxelsize);
voxels=[voxels,...
    1/max(lookuptable(:,5))*ones([size(voxels,1),numOfMats])];

% voxel=[*ones([size(lookuptable,1),max(lookuptable(:,5)])];



winner=zeros([size(all_data_T,1),1]);
% voxel angle,range,P(m1),P(m2),P(m3)


for i=1:size(all_data,1)
    point=all_data(i,:);
    
    angle_bin=round(point(1)/(angle_inc/2))*angle_inc/2;
    range_bin=round(point(3)/(range_inc/2))*range_inc/2;
    possible_mats=find(lookuptable(:,1)>=angle_bin-angle_inc/2 & ...
                       lookuptable(:,1)< angle_bin+angle_inc/2 & ...
                       lookuptable(:,2)>=range_bin-range_inc/2 & ...
                       lookuptable(:,2)< range_bin+range_inc/2);
    sizeofPosmats=size(possible_mats,1);
    
    %this holds the probs of each material, initialised to one and only set
    %if there are 3 contending possibilities
    PerMat_valsOFnormpdf=zeros([1,numOfMats]);
    valsOFnormpdf=zeros([sizeofPosmats,1]);
    
    %if there is only one material pdf then it is that material
    if sizeofPosmats==1
        winner(i)=lookuptable(possible_mats,5);               
        PerMat_valsOFnormpdf(lookuptable(possible_mats,5))=1;
        
    %if there is more than one then compare them
    elseif sizeofPosmats>1  
        if sizeofPosmats==2
            a=1;
        end
%         figure(5);
%         cla('reset')       
        
        for poss_mat_index=1:size(possible_mats,1)
            %get the val on the pdf for this angle range and range range 
            valsOFnormpdf(poss_mat_index)=...
                normpdf(point(2),...
                lookuptable(possible_mats(poss_mat_index),3),...
                lookuptable(possible_mats(poss_mat_index),4));               

            %figure(5)
%             plot(0:0.01:1,normpdf(0:0.01:1,...
%                 lookuptable(possible_mats(poss_mat_index),3),...
%                 lookuptable(possible_mats(poss_mat_index),4)),currentcolor(lookuptable(possible_mats(poss_mat_index),5))); 
%             hold on
            
            %only set this if there is a pdf for each material, otherwise
            %they are all 1, as in basically add no new information.
%             if size(possible_mats,1)==numOfMats
                PerMat_valsOFnormpdf(lookuptable(possible_mats(poss_mat_index),5))=valsOFnormpdf(poss_mat_index);

%             end
        end
        
        
        
        [nothing,index]=max(valsOFnormpdf);
        winner(i)=lookuptable(possible_mats(index),5);
        
%         figure(5);       
%         title(['Classification is: ',num2str(winner(i)),' and correct is: ',num2str(all_data_T(i)) ]);
%         legend('Mat 1','Mat 2','Mat 3');        
%         plot(point(2),0,'k*')
%         pause(1)

        
    end
    
    %fix the PerMat_valsOFnormpdf so there is a prob for each value (make
    %it 1/3 of the smallest value
    try PerMat_valsOFnormpdf(PerMat_valsOFnormpdf<eps)=min(PerMat_valsOFnormpdf(PerMat_valsOFnormpdf>eps))/3;
    catch
        %there may be the case where there is no info about any materials so we can say all 1
%         keyboard;
        PerMat_valsOFnormpdf(:)=1;
        
    end 
 
% temp_voxel=putinVoxels_gp(all_data_xyz(i,:),voxelsize); 
temp_voxel=round(all_data_xyz(i,:)/voxelsize);
%the voxels to update
updatevoxel=find(voxels(:,1)==temp_voxel(1) & voxels(:,2)==temp_voxel(2) & voxels(:,3)==temp_voxel(3),1);
if size(updatevoxel,1)>0
    PerMat_valsOFnormpdf=PerMat_valsOFnormpdf/sum(PerMat_valsOFnormpdf);
    voxels(updatevoxel,4:3+numOfMats)=voxels(updatevoxel,4:3+numOfMats).*PerMat_valsOFnormpdf;
    voxels(updatevoxel,4:3+numOfMats)=voxels(updatevoxel,4:3+numOfMats)/sum(voxels(updatevoxel,4:3+numOfMats));
    %really should a voxel if this is the case
else
keyboard
end
end


    
    M1_index=find(voxels(:,4)>0.5);
    M2_index=find(voxels(:,5)>0.5);
    M3_index=find(voxels(:,6)>0.5);
    figure
    plot3(voxels(M1_index,1),voxels(M1_index,2),voxels(M1_index,3),'r.');   
    hold on;plot3(voxels(M2_index,1),voxels(M2_index,2),voxels(M2_index,3),'g.');
    hold on;plot3(voxels(M3_index,1),voxels(M3_index,2),voxels(M3_index,3),'b.');