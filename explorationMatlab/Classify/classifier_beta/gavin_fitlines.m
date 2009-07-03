%% normalisedata
%
% *Description:* This fits provavilitys to ranges of angles and ranges
% making a PDF over the intensity and basically just outputs the dataset
% and the ansers for the classifier
%

function gavin_fitlines(classifierdata,plotnormfit,plotpolyfit)
% load gavin_classifierdata_rays130to300_mu15mm_voxsize10mm.mat
% global classifierdata

error('problems in loop')

figure(4)
if nargin<3
    plotpolyfit=false;
    if nargin<2
        plotnormfit=true;
    end    
end
angleinc=10;
minangle=0;
maxangle=85;
range_increment=0.1;

%go through each material
for material_num=1:size(classifierdata,2)
    all_sigmahat(material_num).val=[];
    all_muhat(material_num).val=[];
    angle=classifierdata(material_num).val(:,1);
    intensity=classifierdata(material_num).val(:,2);
    range=classifierdata(material_num).val(:,3);

    %go through each range increment
    for range_temp=min(range):range_increment:max(range)       
        index=find((range>=range_temp&...
        range<=range_temp+range_increment));
        if size(index,1)==0
            continue
        end
        
        all_muhat_temp=[];
        all_sigmahat_temp=[];               

        anglerange=minangle:angleinc:maxangle;
        all_muhat_temp=inf*ones([size(anglerange,2),1]);
        all_sigmahat_temp=inf*ones([size(anglerange,2),1]);       
        count=1;
        
        %go through each angle increment
        for i=anglerange
            index2=find(angle(index)>i-angleinc/2 & angle(index)<i+angleinc/2);
            if size(index2,1)>1
                [muhat,sigmahat] = normfit(intensity(index2));
                all_muhat_temp(count)=muhat;
                all_sigmahat_temp(count)=sigmahat;
            end       
            count=count+1;
        end
        
        %we have defined 4 possible colours here
        
        if material_num==1 %rusted metal
            currentcolor='r';
        elseif material_num==2 %wood metal
            currentcolor='g';
        elseif material_num==3 %wood on roof
            currentcolor='b';
        elseif material_num==4 %shinny metal
            currentcolor='k';

        end
            
        %plot
        try 
            figure(4);
            if plotnormfit
                index2=find(all_muhat_temp<inf);
                plot3(anglerange(index2),all_muhat_temp(index2),range_temp*ones([1,size(index2,1)]),'color',currentcolor,'marker','*','markersize',8,'linestyle','none');hold on;
                plot3(anglerange(index2),all_muhat_temp(index2)+all_sigmahat_temp(index2),range_temp*ones([1,size(index2,1)]),'color',currentcolor,'marker','.','markersize',4,'linestyle','none');hold on;
                plot3(anglerange(index2),all_muhat_temp(index2)-all_sigmahat_temp(index2),range_temp*ones([1,size(index2,1)]),'color',currentcolor,'marker','.','markersize',4,'linestyle','none');hold on;
            end
        end
                
        
        if plotpolyfit
            figure(5)
            xdataforpolyfit=[-90:1:90];
            [p] = polyfit(angle(index),intensity(index),2);
            f = polyval(p,xdataforpolyfit);
            plot3(xdataforpolyfit,f,(range_temp+range_increment/2)*ones([size(xdataforpolyfit),1]),'color',currentcolor);
            hold on
            plot3(angle(index),intensity(index),range(index),'color',currentcolor,'marker','.','markersize',4,'linestyle','none')          
        end

        try 
            all_muhat(material_num).val=[all_muhat(material_num).val;all_muhat_temp];
        catch
            keyboard
        end
        all_sigmahat(material_num).val=[all_sigmahat(material_num).val;all_sigmahat_temp];

        xlabel('angle');
        ylabel('intensity over autogain');
        zlabel('range');
        axis([minangle,maxangle,min(intensity),max(intensity),min(range),max(range)]);
    end

    %this is the index of the matrix of mean and sigma
    classifierdata(material_num).lookup.range=min(range):range_increment:max(range);
    classifierdata(material_num).lookup.angle=anglerange;
    classifierdata(material_num).lookup.mean=all_muhat(material_num).val;
    classifierdata(material_num).lookup.sig=all_sigmahat(material_num).val;
 
end