function [ClassifiedData] = Block_Classifier(P_Data, I_Data) 

% P_Data is for the points data
% I_Data is for the intesity data

warning('off');
load('Classification_Criteria.mat');

%% Flags
do_smoothing = 0; %Set to 1 to do median filtering to smooth the classification result
draw_image = 0; %Set to 1 to draw an 2D image of the PC
draw_PC = 0; %Set to 1 to draw the PC
display_ID_edgeim = 0; %displays an image of the intensity data and an image of teh edges found in the intensity image
draw_image_with_edge = 0; %displays the images with the found edges marked on them

run_classifier = 1; %runs the classifier -used 0 when ClassifiedData already exists
replace_hoks_false_intensity_data = 1; %This  replaces the hokuyo's false data with the intensity reading from the ray next to the non-returned  ray but on the same scan

%____________LOOK AT ME!!!!_______________
classiy_scan_range_start = 1;
classiy_scan_range_end = size(P_Data,1);
%_________________________________________

colordef white;

%% The hokuyo uses the last data for a ray if no return is recieved - This
% causes a problem with edge detection. This cell replaces the hokuyo's
% false data with the intensity reading from the ray next to the non-returned 
% ray but from the same sensing data set - so in other words, more false data, but more
% realistic false data
if replace_hoks_false_intensity_data == 1;
    I_DataFlaseDataFixed = I_Data;
    last_known_good_data(size(I_DataFlaseDataFixed,2)) = 0;
    last_known_good_data(:) = I_DataFlaseDataFixed(1,1);

    for i = 2:size(I_DataFlaseDataFixed,2);
        for j = 2:size(I_DataFlaseDataFixed,1);
            if I_DataFlaseDataFixed(j,i) == last_known_good_data(i)
                I_DataFlaseDataFixed(j,i) = I_DataFlaseDataFixed(j,i-1);
            else 
                last_known_good_data(i) = I_DataFlaseDataFixed(j,i);
            end
        end
    end
end

if replace_hoks_false_intensity_data == 1;
    I_DatatoUse = I_DataFlaseDataFixed;
else
    I_DatatoUse = I_Data;
end

%% Edge detecting in I_Data image
% --- Method to use ---
% edge_finding_method_to_use = 'sobel';
% edge_finding_method_to_use = 'prewitt';
  edge_finding_method_to_use = 'roberts';
% edge_finding_method_to_use = 'log';
% edge_finding_method_to_use = 'zerocross';
% edge_finding_method_to_use = 'canny';

InData4Image = I_DatatoUse/max(max(I_DatatoUse));
image_being_used = InData4Image;

%finds edges in the image
Iedges = edge(image_being_used, edge_finding_method_to_use);

%displays images
if display_ID_edgeim == 1
    figure, imshow(image_being_used);
    figure, imshow(Iedges);
end

%% Init's classification result matrix and does classify
if run_classifier == 1
    ClassifiedData = I_DatatoUse; % creats a variable of the correct size - one classifcation per point
    ClassifiedData(:,:) = 8; % sets all points classification to 8 - 'don't know'
    ClassifiedDatawEdges = I_DatatoUse; % creats a variable of the correct size - one classifcation per point
    ClassifiedData(:,:) = 8; % sets all points classification to 8 - 'don't know'
    
    %% Sends data to the classifier one scan_set at a time and creates a matrix of classification results - one for each point
    for i = classiy_scan_range_start:classiy_scan_range_end  
    if mod(i,20) == 0 % puts something in debug so I know its working
        disp([int2str(ceil((i/(classiy_scan_range_end-classiy_scan_range_start))*100)), '% complete']);
    end
        try % I use try as sometimes the classifier crashes for an unknown reason
            found_lines = Classifier(P_Data(i,:,:), I_DatatoUse(i,:,:), 1, Iedges,HParas); % this one uses edges from I_Data Image
            % This creates the classfier output matrix. The matrix is in the same
            % format as P_Data, I_Data, etc      
            number_of_lines = size(found_lines.line_start_end_points_smoothed,1); 
            for j = 1:number_of_lines
                if found_lines.line_start_end_points_smoothed(j,1) > 0
                    number_of_points_classified_by_this_line = found_lines.line_start_end_points_smoothed(j,2)-found_lines.line_start_end_points_smoothed(j,1)+1;
                    for k = 1:number_of_points_classified_by_this_line
                        ClassifiedData(i,(found_lines.line_start_end_points_smoothed(j,1)+k-1)) = found_lines.classifier_output(j);
                    end 
                end
            end
        catch
            display('Error in block classifier');
            lasterr
        end
        

    end
end

%% Put edges (found from intensity data) on the Classified data 
for i = 1:size(Iedges,1)
    for j = 1:size(Iedges,2)
        if Iedges(i,j) == 1
            ClassifiedDatawEdges(i,j) = 9;
        else
            ClassifiedDatawEdges(i,j) = ClassifiedData(i,j);
        end
    end
end

%% Data Smoothing via Median Filtering
if do_smoothing == 1
    %Median filtering
    filter_level = 7;
    ClassifiedDataSmoothed=ClassifiedData/max(max(ClassifiedData)); % pre scaler
    ClassifiedDataSmoothed = medfilt2(ClassifiedDataSmoothed,[filter_level filter_level]); % the filter
    ClassifiedDataSmoothed=ClassifiedDataSmoothed*max(max(ClassifiedData)); %post scaler
    
    % Put edges (found from intensity data) on the Smoothed Classified data 
    for i = 1:size(Iedges,1)
        for j = 1:size(Iedges,2)
            if Iedges(i,j) == 1
                ClassifiedDataSmoothedwEdges(i,j) = 9;
            else
                ClassifiedDataSmoothedwEdges(i,j) = ClassifiedDataSmoothed(i,j);
            end
        end
    end
end

%% Point Cloud 
if draw_PC == 1
    %Coloured (by classification) point cloud
    if draw_image_with_edge == 1
        data_to_display = ClassifiedDatawEdges;
    else
        data_to_display = ClassifiedData;
    end
    output_colours = [1 1 0; 1 0 1; 0 1 1; 1 0 0; 0 1 0; 0 0 1; 0 0 0; 1 1 1; 0.8 0.8 0.8];
    figure();
    view(-90,-90);
    hold on;
    for i = 1:size(data_to_display,1)
        for j = 1:size(output_colours,2)% size(data_to_display,2)
            for k=1:9
                if k~=7
                    a=find(data_to_display(i,:)==k);
                    hold on;
                    plot3(P_Data(i,a,1),P_Data(i,a,2),P_Data(i,a,3),'color', output_colours(j+1,:),'linestyle','none','marker','.','markersize',2);
                end
            end
        end 
    end
    drawnow;
end

%% Image of PC
if draw_image == 1
    if draw_image_with_edge ==1
        data_to_display = ClassifiedDatawEdges;
    else
        data_to_display = ClassifiedData;
    end
    figure();          
    % ungrouped
    %colmap = [[1 1 0]; [1 0 1]; [0 1 1]; [1 0 0]; [0 1 0]; [0 0 1]; [0 0 0]; [1 1 1]; [.8 .8 .8]];
    % grouped in metal - non-metal
    colmap = [[.8 .8 .8]; [.8 .8 .8]; [.8 .8 .8]; [.8 .8 .8]; [.8 .8 .8]; [1 1 0]; [1 1 0]; [1 1 1]; [0 1 0]];
    
    colormap(colmap)
    image(data_to_display);
end

warning('on');
