function Untitled1()
%based on SBRwork
close all
data=imread('normedIntensity_grid1.tif');
%make it one color
 data=squeeze(data(:,:,1));
%or could average across all colors
%data=squeeze((data(:,:,1)+data(:,:,2)+data(:,:,3))/3);
figure
imshow(data)

% for i=3:5
for i=5
    %median filtering on original image    
    figure    
    subplot(2,1,1);
    Medfiltdata=medfilt2(data,[i i]);
    imshow(Medfiltdata )

    subplot(2,1,2);
    K = imadjust(Medfiltdata,[156/255 188/255],[]);
    imshow(K )

    %endge on the original and the changed intensity image
    figure
    subplot(2,1,1)
    BW = edge(Medfiltdata,'canny');
    imshow(BW)
    
    subplot(2,1,2)
    BW2_K=edge(K,'canny');
    imshow(BW2_K)
    
    %image on the to BW image and the intensity to BW
    figure
    subplot(2,1,1)
    BW_roi = roicolor(Medfiltdata,128,255);
    BW_roi = edge(uint8(BW_roi),'canny');
    imshow(BW)
   
    subplot(2,1,2)
    BW2_roi = roicolor(K,128,255);
    BW2_roi = edge(uint8(BW2_roi),'canny');
    imshow(BW2_roi);   
end


BW2_roi



%region of interest all greater than 1 go to 1
BWdata = roicolor(data,1,255);
figure
imshow(BWdata )


filterData=BWdata;
filtersize=3; 
numneeded=3;
for i=1:filtersize:size(data,1)-filtersize
    for j=1:filtersize:size(data,2)-filtersize
        if size(find(filterData(i:i+filtersize-1,j:j+filtersize-1)>0,numneeded),1)>=numneeded
            filterData(i:i+filtersize-1,j:j+filtersize-1)=1;
        end
    end
end
 figure          
imshow(filterData)

%inverse
InverseBWdata=abs(BWdata-1);
figure
imshow(InverseBWdata)



%median filtering
MedfiltBWdata=medfilt2(BWdata,[4 4]);
figure
imshow(MedfiltBWdata )

%wiener2 filter
WiernerBWdata = wiener2(BWdata,[2 2]);
figure
imshow(WiernerBWdata )

InverseWiernerBWdata=abs(WiernerBWdata-1);

dim = size(InverseWiernerBWdata)
col = round(dim(2)/2);
row = min(find(InverseWiernerBWdata(:,col)))
boundary = bwtraceboundary(InverseWiernerBWdata,[row, col],'S');
hold on;
plot(boundary(:,2),boundary(:,1),'g','LineWidth',3);


filterData=InverseWiernerBWdata;
filtersize=10; 
numneeded=30;
for i=1:filtersize:size(data,1)-filtersize
    for j=1:filtersize:size(data,2)-filtersize
        if size(find(filterData(i:i+filtersize-1,j:j+filtersize-1)>0,numneeded),1)>=numneeded
            filterData(i:i+filtersize-1,j:j+filtersize-1)=1;
        end
    end
end
 figure          
imshow(filterData)

dim = size(filterData);
col = round(dim(2)/2);
row = min(find(filterData(:,col)));
boundary = bwtraceboundary(filterData,[row, col],'S');
hold on;
plot(boundary(:,2),boundary(:,1),'r','LineWidth',3);

%final
figure (9)
imshow(data)
hold on
plot(boundary(:,2),boundary(:,1),'r','LineWidth',3);
figure (9)

% BW1 = edge(WiernerBWdata,'sobel');
% BW2 = edge(WiernerBWdata,'canny');
% figure, imshow(BW1)
% figure, imshow(BW2)

keyboard
