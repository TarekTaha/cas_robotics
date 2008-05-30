%% WHEN USING STEVENS/GAVINS MATLAB LASER DATA COLLECTION

function [found_lines] = Classifier(PointData, IntensityData, RangeData, Scan_to_Class, Iedges) % this one uses edges from IntensityData Image

if (exist('Iedges','var')) == 0 % this is so you can call Classifier without Iedges
    Iedges(size(PointData,1),size(PointData,2)) = 0;
end

main_plot_figure = 1;

%% WHEN USING MY GET LASER DATA OR PLY FILES
% function Classifier() 
% global scans_cart
% global found_lines

%% TURNS PLOTTING ON OR OFF
dont_plot = 1; % 0 to plot
erase_mode = 'xor'; %'xor' speeds up plots but makes them a bit uglier
wait_for_clicks = 1;% 0 to make classifier wait for mouse clicks to continue

%% Change data from Stevens format to my format
    %range
    scans_cart.rangeX = PointData(Scan_to_Class,:,2)'*1000;
    scans_cart.rangeY = PointData(Scan_to_Class,:,3)'*1000;
    scans_cart.rangeZ = PointData(Scan_to_Class,:,1)'*1000;
    %intenstiy
    size_data = size(IntensityData, 1);
    angle = size_data * 0.351 * 2;
    theta = ((-angle/2)/180)*pi():(0.352*2/180)*pi():((+angle/2)/180)*pi();
    scans_cart.intensityX(Scan_to_Class,:) = sin(theta(Scan_to_Class)) * IntensityData(Scan_to_Class,:);
    scans_cart.intensityY(Scan_to_Class,:) = cos(theta(Scan_to_Class)) * IntensityData(Scan_to_Class,:);
    scans_cart.intensityX = scans_cart.intensityX(Scan_to_Class,:)';
    scans_cart.intensityY = scans_cart.intensityY(Scan_to_Class,:)';

%% PLOT THE LASER DATA
if dont_plot == 0
    colordef black;
    figure(main_plot_figure);
    %set(1,'name','Point Cloud','Position', [ 570 530 600 400 ]);
    %axis([-800 600 1000 1800 -500 200]);
    %hold off;
    %scatter3(scans_cart.rangeX,scans_cart.rangeY, scans_cart.rangeZ, '.', 'markeredgecolor', [0.75 0.75 0.75],'EraseMode',erase_mode );
    plot3(scans_cart.rangeX,scans_cart.rangeY, scans_cart.rangeZ, '.', 'linestyle','none', 'markersize',1, 'markeredgecolor', [0.75 0.75 0.75],'EraseMode',erase_mode);    
    view(-180,180)
    hold on;
    %axis equal;
    %axis ([ -1000 700 1250 1750 -700 700 ])
end

%% FIND ALL FLAT SURFACES IN THE LASER DATA
    % Initalise variables
    theta_between_rays = 0.351 * pi/180; %angle between laser rays;
    line_segment_smoothing_threshold = 0.5; %allows adjacent lines to be joined if there gradients are similar 
    line_join_dist_threshold = 30; %allows adjacent lines to be joined if they are located near each other 
    min_number_of_points_for_a_line = 10; %min number of points that are needed for a line to regester
    number_of_points = size(scans_cart.rangeX, 1); %number of range points and intensity points. number of rays used is double this
    
    found_lines.line_start_end_points = []; %list of the start and end ray numbers for all lines
    found_lines.line_start_end_points_smoothed = []; %list of the start and end ray numbers for all smoothed lines 
    found_lines.found_lines_gradients = []; % list of the gradients of the found lines
    found_lines.number_of_lines_smoothed = 0;
    
%% TEST FOR LINES (RANSAC) 

    % This function creats a poly fit of x number of points and sees if the
    % next point lies on that line
    max_line_length = 3500; % maximum physical length of any line - mm Helps solve the problem of lines that go back to the scanner origin
    max_points_on_line = 14; %roughly limits number of points to this number - could be upto +30%
    number_of_lines = 0;
    line_ended = 1;
    number_of_points_on_line = 2;

% ---------------------------------------
%     %SPEED BOOST This makes the lines a little less acurate but speed up the classifier
    use_points_set_sizes_of = 8+(floor(rand()*6));
    on_line_threshold = 70; %allows noisey points to be considered to be on the line 
    line_parameters = polyfit(scans_cart.rangeX(3-number_of_points_on_line:3),scans_cart.rangeY(3-number_of_points_on_line:3),1);
    for i = 3:number_of_points
        if mod(i,use_points_set_sizes_of) == 0 %This makes the lines a little less acurate but speed up the classifier
            line_parameters = polyfit(scans_cart.rangeX(i-number_of_points_on_line:i),scans_cart.rangeY(i-number_of_points_on_line:i),1);
        end
    
    %SLOWER BUT MORE ACCURATE - But emphasises RANSAC's vertical line problem       
%     on_line_threshold = 20; %allows noisey points to be considered to be on the line 
%     for i = 3:number_of_points
%         line_parameters = polyfit(scans_cart.rangeX(i-number_of_points_on_line:i),scans_cart.rangeY(i-number_of_points_on_line:i),1);
% ---------------------------------------
        d(i-1) = scans_cart.rangeY(i) - (line_parameters(1)*scans_cart.rangeX(i) + line_parameters(2));
        if abs(d(i-1)) <= on_line_threshold & number_of_points_on_line < max_points_on_line & Iedges(Scan_to_Class,i-2) ~= 1
            if (line_ended == 1)
                found_lines.line_start_end_points = [found_lines.line_start_end_points ; i-2 , 0]; % line START point
                number_of_lines = number_of_lines+1;
            end
            line_ended = 0;   
            number_of_points_on_line = number_of_points_on_line + 1;
        else
            if (line_ended == 0)
                found_lines.line_start_end_points(end,2) = i-2; % line END point
                number_of_points_on_line = 2;
            end
            line_ended = 1;
        end
    end
    
    if (line_ended == 0) & (number_of_points_on_line >= min_number_of_points_for_a_line)
        found_lines.line_start_end_points(end,2) = i; % line END point
    else
        number_of_lines = number_of_lines - 1; %last row is not a true line
    end

%plots originals start/end point markers
% if dont_plot == 0
%   for i = 1:number_of_lines
%     plot3(scans_cart.rangeX(found_lines.line_start_end_points(i,1)),scans_cart.rangeY(found_lines.line_start_end_points(i,1)), scans_cart.rangeZ(found_lines.line_start_end_points(i,1)),'xy','markersize',15,'EraseMode',erase_mode )
%     plot3(scans_cart.rangeX(found_lines.line_start_end_points(i,2)),scans_cart.rangeY(found_lines.line_start_end_points(i,2)), scans_cart.rangeZ(found_lines.line_start_end_points(i,2)),'xy','markersize',15,'EraseMode',erase_mode )
%   end
% end
%waitforbuttonpress()

    %Smooths out detected lines
    for i = 1:number_of_lines
        clear lines;
        line_parameters = polyfit(scans_cart.rangeX(found_lines.line_start_end_points(i,1):found_lines.line_start_end_points(i,2)),scans_cart.rangeY(found_lines.line_start_end_points(i,1):found_lines.line_start_end_points(i,2)),1);
        lines(1,:) = scans_cart.rangeX(found_lines.line_start_end_points(i,1):found_lines.line_start_end_points(i,2)); % fitted line X values
        lines(2,:) = line_parameters(1)*scans_cart.rangeX(found_lines.line_start_end_points(i,1):found_lines.line_start_end_points(i,2)) + line_parameters(2); % fitted line Y values
        gradient_fit_line(i) = (lines(2,1)-lines(2,end))/(lines(1,1)- lines(1,end));
    end
    gradient_fit_line(end+1) = 10000; % used for termination

    found_lines.number_of_lines_smoothed = 1;
    temp_counter = 0;
    for i = 1:number_of_lines-1
        gradient_test = abs(gradient_fit_line(i) - gradient_fit_line(i+1));
        point_proximity_test = (( (scans_cart.rangeX(found_lines.line_start_end_points(i,2))-scans_cart.rangeX(found_lines.line_start_end_points(i+1,1)))^2 + (scans_cart.rangeY(found_lines.line_start_end_points(i,2))-scans_cart.rangeY(found_lines.line_start_end_points(i+1,1)))^2 )^(1/2));
        size_of_line_test = found_lines.line_start_end_points(i,2) - found_lines.line_start_end_points(i-temp_counter,1);
        length_of_line_test = ((scans_cart.rangeX(found_lines.line_start_end_points(i,1)) - scans_cart.rangeX(found_lines.line_start_end_points(i,2)))^2 + (scans_cart.rangeY(found_lines.line_start_end_points(i,1)) - scans_cart.rangeY(found_lines.line_start_end_points(i,2)))^2 + (scans_cart.rangeZ(found_lines.line_start_end_points(i,1)) - scans_cart.rangeZ(found_lines.line_start_end_points(i,2)))^2)^(1/2);
        if gradient_test <= line_segment_smoothing_threshold &  point_proximity_test <= line_join_dist_threshold  & size_of_line_test < max_points_on_line
            temp_counter = temp_counter + 1;
        else
            if abs((found_lines.line_start_end_points(i-temp_counter,1)) - (found_lines.line_start_end_points(i,2))) >= min_number_of_points_for_a_line & length_of_line_test <= max_line_length
                found_lines.line_start_end_points_smoothed(found_lines.number_of_lines_smoothed,1) = found_lines.line_start_end_points(i-temp_counter,1);
                found_lines.line_start_end_points_smoothed(found_lines.number_of_lines_smoothed,2) = found_lines.line_start_end_points(i,2);
                found_lines.number_of_lines_smoothed = found_lines.number_of_lines_smoothed + 1;
                temp_counter = 0;
            end
        end
    end
    found_lines.number_of_lines_smoothed = found_lines.number_of_lines_smoothed-1;

    %Draws on detected lines and SHOULD check for any none valid lines (lines that go through the lasers (0,0)
    for i = 1:found_lines.number_of_lines_smoothed
        clear lines;
        line_parameters = polyfit(scans_cart.rangeX(found_lines.line_start_end_points_smoothed(i,1):found_lines.line_start_end_points_smoothed(i,2)),scans_cart.rangeY(found_lines.line_start_end_points_smoothed(i,1):found_lines.line_start_end_points_smoothed(i,2)),1);
        lines(1,:) = scans_cart.rangeX(found_lines.line_start_end_points_smoothed(i,1):found_lines.line_start_end_points_smoothed(i,2)); % fitted line X values
        lines(2,:) = line_parameters(1)*scans_cart.rangeX(found_lines.line_start_end_points_smoothed(i,1):found_lines.line_start_end_points_smoothed(i,2)) + line_parameters(2); % fitted line Y values
        lines(3,:) = scans_cart.rangeZ(found_lines.line_start_end_points_smoothed(i,1):found_lines.line_start_end_points_smoothed(i,2));
        found_lines.found_lines_gradients = [found_lines.found_lines_gradients; line_parameters(1)];
        %waitforbuttonpress();    

% plots found lines
if dont_plot == 0
    figure(main_plot_figure);
    line(i) = plot3(lines(1,:),lines(2,:),lines(3,:),'c','linestyle','-','EraseMode',erase_mode );
%     plot3(scans_cart.rangeX(found_lines.line_start_end_points_smoothed(i,1)),scans_cart.rangeY(found_lines.line_start_end_points_smoothed(i,1)),scans_cart.rangeZ(found_lines.line_start_end_points_smoothed(i,1)),'oc','markerfacecolor','c','markersize',1.5,'EraseMode',erase_mode );
%     plot3(scans_cart.rangeX(found_lines.line_start_end_points_smoothed(i,2)),scans_cart.rangeY(found_lines.line_start_end_points_smoothed(i,2)),scans_cart.rangeZ(found_lines.line_start_end_points_smoothed(i,2)),'oc','markerfacecolor','c','markersize',1.5,'EraseMode',erase_mode );
end
    end
    
%% INITIALISE VARIABLE - THIS ONES' USED IN THE LOOP
    found_lines.perpindicular_intersecting_ray_numbers = [];
    found_lines.would_be_perp_ray = [];
    found_lines.classifier_output = [];
    i=0; % used for the index in the following loop
    
%% LOOP TO TEST ALL LINES
%for i = 1:found_lines.number_of_lines_smoothed % THE OLD LOOPER
while (i<size(found_lines.line_start_end_points_smoothed,1))
    i = i+1;

%% FIND PERPINDICULAR RAYS
    found_lines.perpindicular_intersecting_ray_numbers(i,1) = 0;
    % test to find if intersection of any rays is perpindicularof to the test line  
    for j = found_lines.line_start_end_points_smoothed(i,1):1:found_lines.line_start_end_points_smoothed(i,2)
         ray_grads(j) = (scans_cart.rangeY(j)/scans_cart.rangeX(j)); %tan(pi()/2 + ((number_of_points/2)-j)*theta_between_rays);
        % test if a perpindicular ray exists on the test line
        if abs(1+(ray_grads(j)*found_lines.found_lines_gradients(i))) <= 0.1 %ray within a few degrees of being perpindicular
            found_lines.perpindicular_intersecting_ray_numbers(i,1) = j;
            break;
        end
    end

    % finds the would be perpindicular ray
    found_lines.would_be_perp_ray(i,1) = 0;
    necessary_gradient = (-1/found_lines.found_lines_gradients(i)); 
    necessary_ray_angle = pi()/2 - (pi()/2 + atan(necessary_gradient));% +CW from -x
    if necessary_ray_angle <=0
        necessary_ray_angle = pi()+necessary_ray_angle;
    end
    for j = 1:number_of_points
        ray_angle(j) =  j*theta_between_rays*2 + abs(atan((scans_cart.rangeY(1)/scans_cart.rangeX(1)))); %only every second ray is for range
        if abs(ray_angle(j) - necessary_ray_angle) <= 0.01
        	found_lines.would_be_perp_ray(i,1) = j;%-3; 
        end
    end
    
    %finds out what side of the line segment the would be perp ray is
    if found_lines.would_be_perp_ray(i,1) < found_lines.line_start_end_points_smoothed(i,1)
        found_lines.perpindicular_intersecting_ray_numbers(i,1) = -10; % indicates perp ray is to the low side
    end
    if found_lines.would_be_perp_ray(i,1) > found_lines.line_start_end_points_smoothed(i,2)
        found_lines.perpindicular_intersecting_ray_numbers(i,1) = -20; % indicates perp ray is to the high side
    end

%plot perp lines  
%if dont_plot == 0
%   figure(main_plot_figure)
%   plotx = -500:500;
%   plot(plotx,plotx*necessary_gradient,'y','EraseMode',erase_mode );
%   plot(scans_cart.rangeX(found_lines.would_be_perp_ray(i,1)),scans_cart.rangeY(found_lines.would_be_perp_ray(i,1)),'.b','markersize',20,'EraseMode',erase_mode )  
% end

%% THIS SECTION ROTATES THE TEST LINE TO BE HORIZONTAL
    rotation_angle = -atan(found_lines.found_lines_gradients(i));
    data_range_to_classify =  [found_lines.line_start_end_points_smoothed(i,1)+2, found_lines.line_start_end_points_smoothed(i,2)-2];
    rotated_scan.rangeX = scans_cart.rangeX((data_range_to_classify(1,1):data_range_to_classify(1,2)))*cos(rotation_angle) - scans_cart.rangeY((data_range_to_classify(1,1):data_range_to_classify(1,2)))*sin(rotation_angle);
    rotated_scan.rangeY = scans_cart.rangeX((data_range_to_classify(1,1):data_range_to_classify(1,2)))*sin(rotation_angle) + scans_cart.rangeY((data_range_to_classify(1,1):data_range_to_classify(1,2)))*cos(rotation_angle);
    rotated_scan.intensityX = scans_cart.intensityX((data_range_to_classify(1,1):data_range_to_classify(1,2)))*cos(rotation_angle) - scans_cart.intensityY((data_range_to_classify(1,1):data_range_to_classify(1,2)))*sin(rotation_angle);
    rotated_scan.intensityY = scans_cart.intensityX((data_range_to_classify(1,1):data_range_to_classify(1,2)))*sin(rotation_angle) + scans_cart.intensityY((data_range_to_classify(1,1):data_range_to_classify(1,2)))*cos(rotation_angle);
    
%% MANIPULATE DATA TO GIVE BEST CHANCE FOR CLASSIFICATION
% Manipulation is done based on where on the line the perpindicular ray
% intersects, if it does intersect
% Options are --> Leave as is - Mirror - Mirror + empty region
    rotated_scan.rangeXM = 0; % this is used to hold the manipulated data
    rotated_scan.rangeYM = 0;
    rotated_scan.intensityXM = 0;
    rotated_scan.intensityYM = 0;
    % intialise variables
    size_data = data_range_to_classify(1,2) - data_range_to_classify(1,1) + 1;
    scan_split_point = found_lines.perpindicular_intersecting_ray_numbers(i) - 4; % the four adjsuts as both edges of the line are trimmed by 2 points to remove noise

    % due to tolerances it is possilbe for teh perp ray to be on the edge
    % or just off the edge of the line
    if scan_split_point > data_range_to_classify(1,2)
        scan_split_point = data_range_to_classify(1,2);
    end

    %determines the type of data manipulation required     
    type_of_interaction = 3;% default - do nothing
    % perp ray on the high side - mirror the low
    if ((data_range_to_classify(1,2) - scan_split_point) <= size_data*0.3) & scan_split_point >= 0
        scan_split_point = found_lines.perpindicular_intersecting_ray_numbers(i) - 4; % the four adjsuts as both edges of the line are trimmed by 2 points to remove noise
        type_of_interaction = 1; % perp ray on the high side - mirror the low
    end
    % perp ray on the low side - mirror the high
    if ((scan_split_point - data_range_to_classify(1,1)) <= size_data*0.3) & scan_split_point  >= 0
        scan_split_point = found_lines.perpindicular_intersecting_ray_numbers(i) - 4; % the four adjsuts as both edges of the line are trimmed by 2 points to remove noise
        type_of_interaction = 2;% perp ray on the low side - mirror the high
    end
    % perp ray in the middle - do nothing
    if abs((data_range_to_classify(1,2) - data_range_to_classify(1,1))/2 - scan_split_point) <= 3 & scan_split_point  >= 0 & mean(rotated_scan.intensityY) > 1500
    	scan_split_point = found_lines.perpindicular_intersecting_ray_numbers(i) - 4; % the four adjsuts as both edges of the line are trimmed by 2 points to remove noise
        type_of_interaction = 3;% perp ray in the middle - do nothing
    end    
    
    % perp ray off to the low side - mirror the high
    if  scan_split_point  == -14
        scan_split_point = data_range_to_classify(1,2)-2; % this sets the split point to the lower limit of the scan range
        %type_of_interaction = 2; %Does the best it can without doing anything special
        type_of_interaction = 5;% perp ray not on line but off to low side   
    end
    % perp ray off to the high side - mirror the low
    if  scan_split_point  == -24
        scan_split_point = data_range_to_classify(1,2)-2; % this sets the split point to the upper limit of the scan range
        %type_of_interaction = 1; %Does the best it can without doing anything special
        type_of_interaction = 4;% perp ray not on line but off to high side
    end
       
    % does the manipulation
    switch (type_of_interaction)
        case(1)  % perp ray on the high ray side - mirror the low 
            % writes orignal data from far low to SP as is
            rotated_scan.rangeYM = rotated_scan.rangeY;
            % takes data from the far low and writes it far high up to SP
            for k = (scan_split_point - (data_range_to_classify(1,1)-3))+1:((scan_split_point - (data_range_to_classify(1,1)-3))*2)-1
            	rotated_scan.rangeYM(k) = rotated_scan.rangeY(end-k+(scan_split_point-(data_range_to_classify(1,1)-3))); 
            end
            % the magnitude of X doesn't matter, but, the spacing between the
            % points must be the same post mirroring
            
            rotated_scan.rangeXM = rotated_scan.rangeX;
            for k = (scan_split_point - (data_range_to_classify(1,1)-3))+1:((scan_split_point - (data_range_to_classify(1,1)-3))*2)-1
                add_this_to_mirrored_X = rotated_scan.rangeX(end-k+(scan_split_point-(data_range_to_classify(1,1)-3))+1) - rotated_scan.rangeX(end-k+(scan_split_point-(data_range_to_classify(1,1)-3)));
                rotated_scan.rangeXM(k) = rotated_scan.rangeXM(k-1) + add_this_to_mirrored_X;
            end
            
            % writes orignal data from far low to SP as is
            rotated_scan.intensityYM = rotated_scan.intensityY;
            % takes data from the far low and writes it far high up to SP
            for k = (scan_split_point - (data_range_to_classify(1,1)-3))+1:((scan_split_point - (data_range_to_classify(1,1)-3))*2)-1
            	rotated_scan.intensityYM(k) = rotated_scan.intensityY(end-k+(scan_split_point-(data_range_to_classify(1,1)-3))); 
            end
            % the magnitude of X doesn't matter, but, the spacing between the
            % points must be the same post mirroring
            
            rotated_scan.intensityXM = rotated_scan.intensityX;
            for k = (scan_split_point - (data_range_to_classify(1,1)-3))+1:((scan_split_point - (data_range_to_classify(1,1)-3))*2)-1
                add_this_to_mirrored_X = rotated_scan.intensityX(end-k+(scan_split_point-(data_range_to_classify(1,1)-3))+1) - rotated_scan.intensityX(end-k+(scan_split_point-(data_range_to_classify(1,1)-3)));
                rotated_scan.intensityXM(k) = rotated_scan.intensityXM(k-1) + add_this_to_mirrored_X;
            end
            
        case(2) % perp ray on the low side - mirror the high    
            % takes data from the far high and writes it far low down to SP
            for k = 1:(data_range_to_classify(1,2))-scan_split_point
                rotated_scan.rangeYM(k,1) = rotated_scan.rangeY(end-k+1);
            end
            % writes orignal data from SP to far high as is
            rotated_scan.rangeYM = [rotated_scan.rangeYM; rotated_scan.rangeY(scan_split_point-(data_range_to_classify(1,1))+2+1:end)];
           
            % the magnitude of X doesn't matter, but, the spacing between the
            % points must be the same post mirroring
            rotated_scan.rangeXM((data_range_to_classify(1,2)-2)-scan_split_point+1:((data_range_to_classify(1,2)-2)-scan_split_point)*2+3,1) = rotated_scan.rangeX(scan_split_point-(data_range_to_classify(1,1))+1:end);
            for k = 1:(data_range_to_classify(1,2))-scan_split_point
                add_this_to_mirrored_X = rotated_scan.rangeXM(k+1+(data_range_to_classify(1,2)-2)-scan_split_point) - rotated_scan.rangeXM(k+(data_range_to_classify(1,2)-2)-scan_split_point);
                rotated_scan.rangeXM((data_range_to_classify(1,2))-scan_split_point-k+1,1) = rotated_scan.rangeXM((data_range_to_classify(1,2))-scan_split_point-k+2) - add_this_to_mirrored_X;
            end
          
            % takes data from the far high and writes it far low down to SP
            for k = 1:(data_range_to_classify(1,2))-scan_split_point
                rotated_scan.intensityYM(k,1) = rotated_scan.intensityY(end-k+1);
            end
            % writes orignal data from SP to far high as is
            rotated_scan.intensityYM = [rotated_scan.intensityYM; rotated_scan.intensityY(scan_split_point-(data_range_to_classify(1,1))+2+1:end)];
           
            % the magnitude of X doesn't matter, but, the spacing between the
            % points must be the same post mirroring
            rotated_scan.intensityXM((data_range_to_classify(1,2)-2)-scan_split_point+1:((data_range_to_classify(1,2)-2)-scan_split_point)*2+3,1) = rotated_scan.intensityX(scan_split_point-(data_range_to_classify(1,1))+1:end);
            for k = 1:(data_range_to_classify(1,2))-scan_split_point
                add_this_to_mirrored_X = rotated_scan.intensityXM(k+1+(data_range_to_classify(1,2)-2)-scan_split_point) - rotated_scan.intensityXM(k+(data_range_to_classify(1,2)-2)-scan_split_point);
                rotated_scan.intensityXM((data_range_to_classify(1,2))-scan_split_point-k+1,1) = rotated_scan.intensityXM((data_range_to_classify(1,2))-scan_split_point-k+2) - add_this_to_mirrored_X;
            end
            
        case(3) % perp ray in the middle - do nothing
            rotated_scan.rangeXM = rotated_scan.rangeX; % this is used to hold the manipulated data
            rotated_scan.rangeYM = rotated_scan.rangeY;
            rotated_scan.intensityXM = rotated_scan.intensityX;
            rotated_scan.intensityYM = rotated_scan.intensityY;

        case(4) % perp ray not on line but off to high side
           
            % writes orignal data from far low to SP as is
            rotated_scan.rangeYM = rotated_scan.rangeY;
            % puts in fake centre points
            num_fake_points_to_add = found_lines.would_be_perp_ray(i) - (data_range_to_classify(1,2)-2);
            rotated_scan.rangeYM((scan_split_point - (data_range_to_classify(1,1)-3))+1:(scan_split_point - (data_range_to_classify(1,1)-3))+1+num_fake_points_to_add) = mean(rotated_scan.rangeYM);
            % takes data from the far low and writes it far high up to SP
            for k = (scan_split_point - (data_range_to_classify(1,1)-3))+2+num_fake_points_to_add:((scan_split_point - (data_range_to_classify(1,1)-3))*2)+1+num_fake_points_to_add
            	rotated_scan.rangeYM(k) = rotated_scan.rangeY(end-k+2+(scan_split_point-(data_range_to_classify(1,1)-3))+num_fake_points_to_add); 
            end
            % the magnitude of X doesn't matter, but, the spacing between the
            % points must be the same post mirroring
            rotated_scan.rangeXM = rotated_scan.rangeX;
            rotated_scan.rangeXM((scan_split_point - (data_range_to_classify(1,1)-3))+num_fake_points_to_add:(scan_split_point - (data_range_to_classify(1,1)-3))+num_fake_points_to_add)=0;
            for k = (scan_split_point - (data_range_to_classify(1,1)-3))+2+num_fake_points_to_add:((scan_split_point - (data_range_to_classify(1,1)-3))*2)+1+num_fake_points_to_add
                rotated_scan.rangeXM(k) = rotated_scan.rangeXM((scan_split_point-(data_range_to_classify(1,1)-3))*2-(k-num_fake_points_to_add)+2)*-1;
            end
            
            % writes orignal data from far low to SP as is
            rotated_scan.intensityYM = rotated_scan.intensityY;
            % puts in fake centre points
            rotated_scan.intensityYM((scan_split_point - (data_range_to_classify(1,1)-3))+1:(scan_split_point - (data_range_to_classify(1,1)-3))+1+num_fake_points_to_add) = 1600;
            % takes data from the far low and writes it far high up to SP
            for k = (scan_split_point - (data_range_to_classify(1,1)-3))+2+num_fake_points_to_add:((scan_split_point - (data_range_to_classify(1,1)-3))*2)+1+num_fake_points_to_add
            	rotated_scan.intensityYM(k) = rotated_scan.intensityY(end-k+2+(scan_split_point-(data_range_to_classify(1,1)-3))+num_fake_points_to_add); 
            end
            % the magnitude of X doesn't matter, but, the spacing between the
            % points must be the same post mirroring
            rotated_scan.intensityXM = rotated_scan.intensityX;
            rotated_scan.intensityXM((scan_split_point - (data_range_to_classify(1,1)-3))+num_fake_points_to_add:(scan_split_point - (data_range_to_classify(1,1)-3))+num_fake_points_to_add)=0;
            for k = (scan_split_point - (data_range_to_classify(1,1)-3))+2+num_fake_points_to_add:((scan_split_point - (data_range_to_classify(1,1)-3))*2)+1+num_fake_points_to_add
                rotated_scan.intensityXM(k) = rotated_scan.intensityXM((scan_split_point-(data_range_to_classify(1,1)-3))*2-(k-num_fake_points_to_add)+2)*-1;
            end
            
        case(5) % perp ray not on line but off to low side
            
            % reverses order of data so I can reuse MOST of the code from case(4)
            rotated_scan.rangeXFlipped = [];
            rotated_scan.rangeYFlipped = [];
            rotated_scan.intensityXFlipped = [];
            rotated_scan.intensityYFlipped = [];
            rotated_scan.rangeXFlipped = wrev(rotated_scan.rangeX);
            rotated_scan.rangeYFlipped = wrev(rotated_scan.rangeY);
            rotated_scan.intensityXFlipped = wrev(rotated_scan.intensityX);
            rotated_scan.intensityYFlipped = wrev(rotated_scan.intensityY);
%             rotated_scan.rangeXFlipped(1:end,1) = rotated_scan.rangeX(end:-1:1,1);
%             rotated_scan.rangeYFlipped(1:end,1) = rotated_scan.rangeY(end:-1:1,1);
%             rotated_scan.intensityXFlipped(1:end,1) = rotated_scan.intensityX(end:-1:1,1);
%             rotated_scan.intensityYFlipped(1:end,1) =
%             rotated_scan.intensityY(end:-1:1,1);
            
            % writes orignal data from far low to SP as is
            rotated_scan.rangeYM = rotated_scan.rangeYFlipped;
            % puts in fake centre points - DIFFERENT FROM CASE 4
            num_fake_points_to_add = (data_range_to_classify(1,1)+2)-found_lines.would_be_perp_ray(i);
            rotated_scan.rangeYM((scan_split_point - (data_range_to_classify(1,1)-3))+1:(scan_split_point - (data_range_to_classify(1,1)-3))+1+num_fake_points_to_add) = mean(rotated_scan.rangeYM);
            % takes data from the far low and writes it far high up to SP
            for k = (scan_split_point - (data_range_to_classify(1,1)-3))+2+num_fake_points_to_add:((scan_split_point - (data_range_to_classify(1,1)-3))*2)+1+num_fake_points_to_add
            	rotated_scan.rangeYM(k) = rotated_scan.rangeYFlipped(end-k+2+(scan_split_point-(data_range_to_classify(1,1)-3))+num_fake_points_to_add); 
            end
            % the magnitude of X doesn't matter, but, the spacing between the
            % points must be the same post mirroring
            rotated_scan.rangeXM = rotated_scan.rangeXFlipped;
            rotated_scan.rangeXM((scan_split_point - (data_range_to_classify(1,1)-3))+num_fake_points_to_add:(scan_split_point - (data_range_to_classify(1,1)-3))+num_fake_points_to_add)=0;
            for k = (scan_split_point - (data_range_to_classify(1,1)-3))+2+num_fake_points_to_add:((scan_split_point - (data_range_to_classify(1,1)-3))*2)+1+num_fake_points_to_add
                rotated_scan.rangeXM(k) = rotated_scan.rangeXM((scan_split_point-(data_range_to_classify(1,1)-3))*2-(k-num_fake_points_to_add)+2)*-1;
            end
            
            % writes orignal data from far low to SP as is
            rotated_scan.intensityYM = rotated_scan.intensityYFlipped;
            % puts in fake centre point
            rotated_scan.intensityYM((scan_split_point - (data_range_to_classify(1,1)-3))+1:(scan_split_point - (data_range_to_classify(1,1)-3))+1+num_fake_points_to_add) = 1600;
            % takes data from the far low and writes it far high up to SP
            for k = (scan_split_point - (data_range_to_classify(1,1)-3))+2+num_fake_points_to_add:((scan_split_point - (data_range_to_classify(1,1)-3))*2)+1+num_fake_points_to_add
            	rotated_scan.intensityYM(k) = rotated_scan.intensityYFlipped(end-k+2+(scan_split_point-(data_range_to_classify(1,1)-3))+num_fake_points_to_add); 
            end
            % the magnitude of X doesn't matter, but, the spacing between the
            % points must be the same post mirroring
            rotated_scan.intensityXM = rotated_scan.intensityXFlipped;
            rotated_scan.intensityXM((scan_split_point - (data_range_to_classify(1,1)-3))+num_fake_points_to_add:(scan_split_point - (data_range_to_classify(1,1)-3))+num_fake_points_to_add)=0;
            for k = (scan_split_point - (data_range_to_classify(1,1)-3))+2+num_fake_points_to_add:((scan_split_point - (data_range_to_classify(1,1)-3))*2)+1+num_fake_points_to_add
                rotated_scan.intensityXM(k) = rotated_scan.intensityXM((scan_split_point-(data_range_to_classify(1,1)-3))*2-(k-num_fake_points_to_add)+2)*-1;
            end
    end

%% POLY FITS TO LINES
    % initalise variables
    order = 2; %the order of the poly fits
    
    %poly fit for the range data
    polyfit_coefs_range = polyfit(rotated_scan.rangeXM(:), rotated_scan.rangeYM(:), order);
    fitted_line_vals_range = [polyval(polyfit_coefs_range, rotated_scan.rangeXM(:)) , rotated_scan.rangeXM(:)];
    fitted_line_vals_width = size(fitted_line_vals_range, 2);

    %poly fit for the intensity data
    polyfit_coefs_intensity = polyfit(rotated_scan.intensityXM(:), rotated_scan.intensityYM(:), order);
    fitted_line_vals_intensity = [polyval(polyfit_coefs_intensity, rotated_scan.intensityXM(:)) , rotated_scan.intensityXM(:)];

% plots the actual data and poly fits for the found line
% if dont_plot == 0
% 	 figure(10+i); 
%    hold on;
%    %laser data
%    plot(rotated_scan.rangeXM,rotated_scan.rangeYM,'EraseMode',erase_mode )
%    plot(rotated_scan.intensityXM,rotated_scan.intensityYM,'EraseMode',erase_mode )
%    %poly fit
%    plot(rotated_scan.rangeXM(:), fitted_line_vals_range(:,1), 'color', 'r','EraseMode',erase_mode )
%    plot(rotated_scan.intensityXM(:), fitted_line_vals_intensity(:,1), 'color', 'r','EraseMode',erase_mode )
%    plot(rotated_scan.intensityX,rotated_scan.intensityY,'b','EraseMode',erase_mode )
% end

%% CALCULATE THE REQUIRED RESIDUALS FOR THE POLYs BEING TESTED

    [rubbish size_fitted_line_vals] = size(fitted_line_vals_intensity);
    residual_range = [ rotated_scan.rangeYM(:) - fitted_line_vals_range(:,1)];
    residual_intensity = [ rotated_scan.intensityYM(:) - fitted_line_vals_intensity(:,1)];
    [rubbish size_residual] = size(residual_range);
    mse_range = mse(residual_range(:));
    mse_intensity = mse(residual_intensity(:));
    [mse_size rubbish] = size(mse_range);
    
%plots the residuals and puts up message box with MSE's in it
% if dont_plot == 0
%    figure(20+i); 
%    hold on;
%    plot(residual_range(:),'r','EraseMode',erase_mode );
%    plot(residual_intensity(:),'EraseMode',erase_mode );
%    axis([0 60 -700 400])
%    message = ['MSE Range:', int2str(mse_range),'   MSE Intensity:',int2str(mse_intensity)];
%    msgbox(message,int2str(i));
%    waitforbuttonpress()
% end  

%% Probability Based Classifier
    load('Classification_Criteria.mat');

    lhoods.liki_ratio_varis = [];

    %likelihood based on intensity
    lhoods.I.GSteel = normpdf(mse_intensity, HParas.I.GSteel_mean, HParas.I.GSteel_std);
    lhoods.I.Alumin = normpdf(mse_intensity, HParas.I.Alumin_mean, HParas.I.Alumin_std);
    lhoods.I.GMetal = normpdf(mse_intensity, HParas.I.GMetal_mean, HParas.I.GMetal_std);
    lhoods.I.MSteel = normpdf(mse_intensity, HParas.I.MSteel_mean, HParas.I.MSteel_std);
    lhoods.I.Copper = normpdf(mse_intensity, HParas.I.Copper_mean, HParas.I.Copper_std);
    lhoods.I.DPlyWd = normpdf(mse_intensity, HParas.I.DPlyWd_mean, HParas.I.DPlyWd_std);
    lhoods.I.BCloth = normpdf(mse_intensity, HParas.I.BCloth_mean, HParas.I.BCloth_std);

    %likelihood based on range
    lhoods.R.GSteel = normpdf(mse_range, HParas.R.GSteel_mean, HParas.R.GSteel_std);
    lhoods.R.Alumin = normpdf(mse_range, HParas.R.Alumin_mean, HParas.R.Alumin_std);
    lhoods.R.GMetal = normpdf(mse_range, HParas.R.GMetal_mean, HParas.R.GMetal_std);
    lhoods.R.MSteel = normpdf(mse_range, HParas.R.MSteel_mean, HParas.R.MSteel_std);
    lhoods.R.Copper = normpdf(mse_range, HParas.R.Copper_mean, HParas.R.Copper_std);
    lhoods.R.DPlyWd = normpdf(mse_range, HParas.R.DPlyWd_mean, HParas.R.DPlyWd_std);
    lhoods.R.BCloth = normpdf(mse_range, HParas.R.BCloth_mean, HParas.R.BCloth_std);

    %likelihood based on both 
    lhoods.combined.GSteel = lhoods.I.GSteel * lhoods.R.GSteel;
    lhoods.liki_ratio_varis = [lhoods.liki_ratio_varis; lhoods.combined.GSteel , 1];

    lhoods.combined.Alumin = lhoods.I.Alumin * lhoods.R.Alumin;
    lhoods.liki_ratio_varis = [lhoods.liki_ratio_varis; lhoods.combined.Alumin , 2];

    lhoods.combined.GMetal = lhoods.I.GMetal * lhoods.R.GMetal;
    lhoods.liki_ratio_varis = [lhoods.liki_ratio_varis; lhoods.combined.GMetal , 3];

    lhoods.combined.MSteel = lhoods.I.MSteel * lhoods.R.MSteel;
    lhoods.liki_ratio_varis = [lhoods.liki_ratio_varis; lhoods.combined.MSteel , 4];

    lhoods.combined.Copper = lhoods.I.Copper * lhoods.R.Copper;
    lhoods.liki_ratio_varis = [lhoods.liki_ratio_varis; lhoods.combined.Copper , 5];

    lhoods.combined.DPlyWd = lhoods.I.DPlyWd * lhoods.R.DPlyWd;
    lhoods.liki_ratio_varis = [lhoods.liki_ratio_varis; lhoods.combined.DPlyWd , 6];

    lhoods.combined.BCloth = lhoods.I.BCloth * lhoods.R.BCloth;
    lhoods.liki_ratio_varis = [lhoods.liki_ratio_varis; lhoods.combined.BCloth , 7];

    %order to most likely first
    lhoods.liki_ratio_varis = sortrows(lhoods.liki_ratio_varis, 1);

    %likelihood ratio
    lhoods.ratio = lhoods.liki_ratio_varis(end,1)/lhoods.liki_ratio_varis(end-1,1);

    %Classification
    material_to_class = lhoods.liki_ratio_varis(end,2);
    PClass.Confidence = lhoods.ratio/(1+lhoods.ratio);

    switch(material_to_class)
        case 1
            PClass.Material = 'GSteel';
            guess = 1;
        case 2
            PClass.Material = 'Alumin';
            guess = 2;
        case 3
            PClass.Material = 'GMetal';
            guess = 3;
        case 4
            PClass.Material = 'MSteel';  
            guess = 4;
        case 5
            PClass.Material = 'Copper';
            guess = 5;
        case 6
            PClass.Material = 'DPlyWd';
            guess = 6;
        case 7
            PClass.Material = 'BCloth'; 
            guess = 7;
    end


%%  User Output
    switch (guess)
        case 1
            output = 'Galv Steel ';
            output_color = [1 1 0]; % Yellow
        case 2
            output = 'Aluminium  ';
            output_color = [1 0 1]; % Magenta
        case 3
            output = 'GMetal     '; % Cyan
            output_color = [0 1 1]; 
        case 4
            output = 'Mild Steel ';
            output_color = [1 0 0]; % Red 
        case 5
            output = 'Copper     ';
            output_color = [0 1 0]; % Green 
        case 6
            output = 'Plywood    ';
            output_color = [0 0 1]; % Blue 
        case 7
            output = 'Cloth      ';
            output_color = [0 0 0]; % Black 
        case 8
            output = 'Do not know';
            output_color = [1 1 1]; % WHITE 
    end 
    
% %make every 20th line blue
% if mod(Scan_to_Class,20) == 0
%     output_color = [0 1 1] %CYAN
% end
    
    %creates colour coded figure as user output
if dont_plot == 0
    set(line(i),'color',output_color,'linewidth',5);
    if output_color == [1 1 1] 
        delete(line(i)); %delets the line from memory
        %set(line(i),'color',output_color,'linestyle','none'); %hides the line
    end
end

% if dont_plot == 0
%     figure(30)
%     display_info = ['Line #' , int2str(i), ' - ', output];
%     set(30,'name',display_info,'Position',[ 700 370 350 50 ]);
%     display_info = ['Line #' , int2str(i)];
%     set(1,'name',display_info);
%     plot(1,1,'o','MarkerEdgeColor','k','MarkerFaceColor',output_color,'MarkerSize',10000,'EraseMode',erase_mode );
% end

    % puts the result in an accessible place
    found_lines.classifier_output(i,1)= guess;

%% DEAL WITH NO CLASSIFIED SURFACES BY SPLITTING THEM AND RE-CLASSIFING
    if found_lines.classifier_output(i,1) == 4    
        if (found_lines.line_start_end_points_smoothed(i,2)-found_lines.line_start_end_points_smoothed(i,1))/2 > min_number_of_points_for_a_line
            if wait_for_clicks == 0
                waitforbuttonpress()
            end
            found_lines.line_start_end_points_smoothed(end+1,1) = found_lines.line_start_end_points_smoothed(i,1);
            found_lines.line_start_end_points_smoothed(end,2) = floor(found_lines.line_start_end_points_smoothed(i,1)+((found_lines.line_start_end_points_smoothed(i,2)-found_lines.line_start_end_points_smoothed(i,1))/2));
            found_lines.line_start_end_points_smoothed(end+1,2) = found_lines.line_start_end_points_smoothed(i,2);
            found_lines.line_start_end_points_smoothed(end,1) = floor(found_lines.line_start_end_points_smoothed(i,2)-((found_lines.line_start_end_points_smoothed(i,2)-found_lines.line_start_end_points_smoothed(i,1))/2))+1;
            found_lines.line_start_end_points_smoothed(i,1) = -found_lines.line_start_end_points_smoothed(i,1);
            found_lines.line_start_end_points_smoothed(i,2) = -found_lines.line_start_end_points_smoothed(i,2);
            found_lines.found_lines_gradients(end+1,1) = found_lines.found_lines_gradients(i,1);
            found_lines.found_lines_gradients(end+1,1) = found_lines.found_lines_gradients(i,1);
%draws first of the split lines on
if dont_plot == 0
    figure(main_plot_figure);
    temp_i = length(found_lines.line_start_end_points_smoothed)-1;
    clear lines;
    line_parameters = polyfit(scans_cart.rangeX(found_lines.line_start_end_points_smoothed(temp_i,1):found_lines.line_start_end_points_smoothed(temp_i,2)),scans_cart.rangeY(found_lines.line_start_end_points_smoothed(temp_i,1):found_lines.line_start_end_points_smoothed(temp_i,2)),1);
    lines(1,:) = scans_cart.rangeX(found_lines.line_start_end_points_smoothed(temp_i,1):found_lines.line_start_end_points_smoothed(temp_i,2)); % fitted line X values
    lines(2,:) = line_parameters(1)*scans_cart.rangeX(found_lines.line_start_end_points_smoothed(temp_i,1):found_lines.line_start_end_points_smoothed(temp_i,2)) + line_parameters(2); % fitted line Y values
    lines(3,:) = scans_cart.rangeZ(found_lines.line_start_end_points_smoothed(temp_i,1):found_lines.line_start_end_points_smoothed(temp_i,2));
    line(temp_i) = plot3(lines(1,:),lines(2,:),lines(3,:),'b','linestyle','-','EraseMode',erase_mode );
    %otherside of split
    temp_i = length(found_lines.line_start_end_points_smoothed);
    clear lines;
    line_parameters = polyfit(scans_cart.rangeX(found_lines.line_start_end_points_smoothed(temp_i,1):found_lines.line_start_end_points_smoothed(temp_i,2)),scans_cart.rangeY(found_lines.line_start_end_points_smoothed(temp_i,1):found_lines.line_start_end_points_smoothed(temp_i,2)),1);
    lines(1,:) = scans_cart.rangeX(found_lines.line_start_end_points_smoothed(temp_i,1):found_lines.line_start_end_points_smoothed(temp_i,2)); % fitted line X values
    lines(2,:) = line_parameters(1)*scans_cart.rangeX(found_lines.line_start_end_points_smoothed(temp_i,1):found_lines.line_start_end_points_smoothed(temp_i,2)) + line_parameters(2); % fitted line Y values
    lines(3,:) = scans_cart.rangeZ(found_lines.line_start_end_points_smoothed(temp_i,1):found_lines.line_start_end_points_smoothed(temp_i,2));
    line(temp_i) = plot3(lines(1,:),lines(2,:),lines(3,:),'b','linestyle','-','EraseMode',erase_mode );
    %plots split points
%     plot3(scans_cart.rangeX(found_lines.line_start_end_points_smoothed(end,1)),scans_cart.rangeY(found_lines.line_start_end_points_smoothed(end,1)),scans_cart.rangeZ(found_lines.line_start_end_points_smoothed(end,1)),'oc','markerfacecolor','b','markersize',1.5,'EraseMode',erase_mode );
%     plot3(scans_cart.rangeX(found_lines.line_start_end_points_smoothed(end-1,1)),scans_cart.rangeY(found_lines.line_start_end_points_smoothed(end-1,1)),scans_cart.rangeZ(found_lines.line_start_end_points_smoothed(end-1,1)),'oc','markerfacecolor','b','markersize',1.5,'EraseMode',erase_mode );
%     plot3(scans_cart.rangeX(found_lines.line_start_end_points_smoothed(end,2)),scans_cart.rangeY(found_lines.line_start_end_points_smoothed(end,2)),scans_cart.rangeX(found_lines.line_start_end_points_smoothed(end,2)),'oc','markerfacecolor','b','markersize',1.5,'EraseMode',erase_mode );

end
       end
    end

%% END OF TEST ALL LINES LOOP
    if wait_for_clicks == 0
        waitforbuttonpress()
    end
end
    