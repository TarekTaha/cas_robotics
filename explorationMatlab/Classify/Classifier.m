%% Classifier
% *Description*: This classifies a single scan sweep WHEN USING
% STEVENS/GAVINS MATLAB LASER DATA COLLECTION 
%
%% Function Call
%
% *Inputs:* 
%
% _PointData_ (size, type string) what is its purpose?
%
% _IntensityData_ (size, type string) what is its purpose?
%
% _Scan_to_Class_ (size, type string) what is its purpose?
%
% _Iedges_ (size, type string) what is its purpose?
%
% _HParas_ (size, type string) what is its purpose?
%
% *Returns:* 
%
% _found_lines_ (size, type string) what is its purpose?
%


function [found_lines] = Classifier(PointData, IntensityData, Scan_to_Class, Iedges, HParas) % this one uses edges from IntensityData Image

if (exist('Iedges','var')) == 0 % this is so you can call Classifier without Iedges
    Iedges(size(PointData,1),size(PointData,2)) = 0;
end

%% Variables
theta_between_rays = 0.351 * pi/180; %angle between laser rays;

usegavin_func=1;

%% Change data from Stevens format to my format
%range
scans_cart.rangeX = PointData(Scan_to_Class,:,2)'*1000;
scans_cart.rangeY = PointData(Scan_to_Class,:,3)'*1000;
scans_cart.rangeZ = PointData(Scan_to_Class,:,1)'*1000;

%intenstiy
size_data = size(IntensityData, 1);
angle = size_data * theta_between_rays * 2;
theta = -angle/2:theta_between_rays*2:+angle/2;
scans_cart.intensityX = sin(theta(Scan_to_Class)) * IntensityData(Scan_to_Class,:)';
scans_cart.intensityY = cos(theta(Scan_to_Class)) * IntensityData(Scan_to_Class,:)';

%% FIND ALL FLAT SURFACES IN THE LASER DATA
% Initalise variables    
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
line_ended = true;
number_of_points_on_line = 2; 
on_line_threshold = 70; %allows noisey points to be considered to be on the line        

%polyfit
if usegavin_func %new one
    line_parameters= polyfit_gp(scans_cart.rangeX(3-number_of_points_on_line:3),scans_cart.rangeY(3-number_of_points_on_line:3),1);       
else % old one
    line_parameters =   polyfit(scans_cart.rangeX(3-number_of_points_on_line:3),scans_cart.rangeY(3-number_of_points_on_line:3),1);
end

%use fewer line checks SPEED BOOST This makes the lines a little less acurate but speed up the classifier
 use_points_set_sizes_of = 8+(floor(rand()*6));
% use_points_set_sizes_of = 2+(floor(rand()*6));
%use_points_set_sizes_of = 1; %use all of them makes it much better

for i = 3:number_of_points
    if mod(i,use_points_set_sizes_of) == 0
        if usegavin_func %new one
            line_parameters= polyfit_gp(scans_cart.rangeX(i-number_of_points_on_line:i),scans_cart.rangeY(i-number_of_points_on_line:i),1);
        else % old one
            line_parameters =   polyfit(scans_cart.rangeX(i-number_of_points_on_line:i),scans_cart.rangeY(i-number_of_points_on_line:i),1);
        end
    end

%SLOWER BUT MORE ACCURATE - But emphasises RANSAC's vertical line problem
    d_val = scans_cart.rangeY(i) - (line_parameters(1)*scans_cart.rangeX(i) + line_parameters(2));
    if abs(d_val) <= on_line_threshold && ...
       number_of_points_on_line < max_points_on_line && ...
       Iedges(Scan_to_Class,i-2) ~= 1
        if (line_ended == true)
            found_lines.line_start_end_points = [found_lines.line_start_end_points ; i-2 , 0]; % line START point
            number_of_lines = number_of_lines+1;
            line_ended = false;
        end   
        number_of_points_on_line = number_of_points_on_line + 1;
    else
        if (line_ended == false)
            found_lines.line_start_end_points(end,2) = i-2; % line END point
            number_of_points_on_line = 2;
            line_ended = true;
        end
    end
end

if (line_ended == false) && (number_of_points_on_line >= min_number_of_points_for_a_line)
    found_lines.line_start_end_points(end,2) = i; % line END point
else
    number_of_lines = number_of_lines - 1; %last row is not a true line
end

%% Smooths out detected lines
gradient_fit_line=[zeros([1,number_of_lines]),inf]; % final value used for termination
for i = 1:number_of_lines
    if usegavin_func %new one
        line_parameters= polyfit_gp(scans_cart.rangeX(found_lines.line_start_end_points(i,1):found_lines.line_start_end_points(i,2)),scans_cart.rangeY(found_lines.line_start_end_points(i,1):found_lines.line_start_end_points(i,2)),1);
    else % old one
        line_parameters =   polyfit(scans_cart.rangeX(found_lines.line_start_end_points(i,1):found_lines.line_start_end_points(i,2)),scans_cart.rangeY(found_lines.line_start_end_points(i,1):found_lines.line_start_end_points(i,2)),1);
    end
    %only need the m value from y=mx+b
    gradient_fit_line(i)=line_parameters(1);
end

found_lines.number_of_lines_smoothed = 1;
temp_counter = 0;
for i = 1:number_of_lines-1
    gradient_test = abs(gradient_fit_line(i) - gradient_fit_line(i+1));
    point_proximity_test = sqrt((scans_cart.rangeX(found_lines.line_start_end_points(i,2))-...
                                 scans_cart.rangeX(found_lines.line_start_end_points(i+1,1)))^2 +...
                                (scans_cart.rangeY(found_lines.line_start_end_points(i,2))-...
                                 scans_cart.rangeY(found_lines.line_start_end_points(i+1,1)))^2 );
    size_of_line_test = found_lines.line_start_end_points(i,2) - found_lines.line_start_end_points(i-temp_counter,1);
    length_of_line_test = sqrt((scans_cart.rangeX(found_lines.line_start_end_points(i,1)) -...
                                scans_cart.rangeX(found_lines.line_start_end_points(i,2)))^2 +...
                               (scans_cart.rangeY(found_lines.line_start_end_points(i,1)) - ...
                                scans_cart.rangeY(found_lines.line_start_end_points(i,2)))^2 + ...
                               (scans_cart.rangeZ(found_lines.line_start_end_points(i,1)) - ...
                                scans_cart.rangeZ(found_lines.line_start_end_points(i,2)))^2);
    if gradient_test <= line_segment_smoothing_threshold && ...
       point_proximity_test <= line_join_dist_threshold  && ...
       size_of_line_test < max_points_on_line
        temp_counter = temp_counter + 1;
    else
        if abs((found_lines.line_start_end_points(i-temp_counter,1)) - (found_lines.line_start_end_points(i,2))) >= ...
           min_number_of_points_for_a_line && ...
           length_of_line_test <= max_line_length
            found_lines.line_start_end_points_smoothed(found_lines.number_of_lines_smoothed,1) = found_lines.line_start_end_points(i-temp_counter,1);
            found_lines.line_start_end_points_smoothed(found_lines.number_of_lines_smoothed,2) = found_lines.line_start_end_points(i,2);
            found_lines.number_of_lines_smoothed = found_lines.number_of_lines_smoothed + 1;
            temp_counter = 0;
        end
    end
end

found_lines.number_of_lines_smoothed = found_lines.number_of_lines_smoothed-1;

%Draws on detected lines and SHOULD check for any none valid lines (lines that go through the lasers (0,0)
found_lines.found_lines_gradients=zeros([found_lines.number_of_lines_smoothed,1]); 
for i = 1:found_lines.number_of_lines_smoothed 
    if usegavin_func %new one
        line_parameters= polyfit_gp(scans_cart.rangeX(found_lines.line_start_end_points_smoothed(i,1):found_lines.line_start_end_points_smoothed(i,2)),scans_cart.rangeY(found_lines.line_start_end_points_smoothed(i,1):found_lines.line_start_end_points_smoothed(i,2)),1);
    else % old one
        line_parameters =   polyfit(scans_cart.rangeX(found_lines.line_start_end_points_smoothed(i,1):found_lines.line_start_end_points_smoothed(i,2)),scans_cart.rangeY(found_lines.line_start_end_points_smoothed(i,1):found_lines.line_start_end_points_smoothed(i,2)),1);
    end      
        
    found_lines.found_lines_gradients(i) = line_parameters(1);

end
    
%% INITIALISE VARIABLE - THIS ONES' USED IN THE LOOP
found_lines.perpindicular_intersecting_ray_numbers = [];
found_lines.would_be_perp_ray = [];
found_lines.classifier_output = [];
i=0; % used for the index in the following loop
    
%% LOOP TO TEST ALL LINES
% for i = 1:1:(size(found_lines.line_start_end_points_smoothed,1)) % DONT KNOW WHY THIS DOSEN'T work
while (i<size(found_lines.line_start_end_points_smoothed,1))
    i = i+1;

%% FIND PERPINDICULAR RAYS
    found_lines.perpindicular_intersecting_ray_numbers(i,1) = 0;
    % test to find if intersection of any rays is perpindicularof to the test line  
    for j = found_lines.line_start_end_points_smoothed(i,1):1:found_lines.line_start_end_points_smoothed(i,2)
         ray_grads = (scans_cart.rangeY(j)/scans_cart.rangeX(j)); %tan(pi/2 + ((number_of_points/2)-j)*theta_between_rays);
        % test if a perpindicular ray exists on the test line
        if abs(1+(ray_grads*found_lines.found_lines_gradients(i))) <= 0.1 %ray within a few degrees of being perpindicular
            found_lines.perpindicular_intersecting_ray_numbers(i,1) = j;
            break;
        end
    end

    % finds the would be perpindicular ray
    found_lines.would_be_perp_ray(i,1) = 0;
    necessary_gradient = (-1/found_lines.found_lines_gradients(i)); 
    necessary_ray_angle = pi/2 - (pi/2 + atan(necessary_gradient));% +CW from -x
    if necessary_ray_angle <=0
        necessary_ray_angle = pi+necessary_ray_angle;
    end
    for j = 1:number_of_points
        ray_angle =  j*theta_between_rays*2 + abs(atan((scans_cart.rangeY(1)/scans_cart.rangeX(1)))); %only every second ray is for range
        if abs(ray_angle - necessary_ray_angle) <= 0.01
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
    if ((data_range_to_classify(1,2) - scan_split_point) <= size_data*0.3) && scan_split_point >= 0
        scan_split_point = found_lines.perpindicular_intersecting_ray_numbers(i) - 4; % the four adjsuts as both edges of the line are trimmed by 2 points to remove noise
        type_of_interaction = 1; % perp ray on the high side - mirror the low
    end
    % perp ray on the low side - mirror the high
    if ((scan_split_point - data_range_to_classify(1,1)) <= size_data*0.3) && scan_split_point >= 0
        scan_split_point = found_lines.perpindicular_intersecting_ray_numbers(i) - 4; % the four adjsuts as both edges of the line are trimmed by 2 points to remove noise
        type_of_interaction = 2;% perp ray on the low side - mirror the high
    end
    % perp ray in the middle - do nothing
    if abs((data_range_to_classify(1,2) - data_range_to_classify(1,1))/2 - scan_split_point) <= 3 &&...
            scan_split_point  >= 0 &&...
            sum(rotated_scan.intensityY)/size(rotated_scan.intensityY,1) > 1500
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
    if usegavin_func; %new one
        polyfit_coefs_range = polyfit_gp(rotated_scan.rangeXM(:), rotated_scan.rangeYM(:), 2);
    else % old one
        polyfit_coefs_range =    polyfit(rotated_scan.rangeXM(:), rotated_scan.rangeYM(:), order);
    end             
    
    fitted_line_vals_range= [polyfit_coefs_range(1)*rotated_scan.rangeXM(:).^2+...
                            polyfit_coefs_range(2)*rotated_scan.rangeXM(:)+...
                            polyfit_coefs_range(3),rotated_scan.rangeXM(:),rotated_scan.rangeXM(:)];
                        
%     fitted_line_vals_width = size(fitted_line_vals_range, 2);

    %poly fit for the intensity data
    if usegavin_func; %new one
        polyfit_coefs_intensity = polyfit_gp(rotated_scan.intensityXM(:), rotated_scan.intensityYM(:), 2);
        fitted_line_vals_intensity = [polyfit_coefs_intensity(1)*rotated_scan.intensityXM(:).^2+...
                                      polyfit_coefs_intensity(2)*rotated_scan.intensityXM(:)+...
                                      polyfit_coefs_intensity(3),rotated_scan.intensityXM(:)];                         
    else % old one
        polyfit_coefs_intensity = polyfit(rotated_scan.intensityXM(:), rotated_scan.intensityYM(:), order);
        fitted_line_vals_intensity = [polyval(polyfit_coefs_intensity, rotated_scan.intensityXM(:)) , rotated_scan.intensityXM(:)];
    end      

%% CALCULATE THE REQUIRED RESIDUALS FOR THE POLYs BEING TESTED

%     size_fitted_line_vals = size(fitted_line_vals_intensity,2);
    residual_range = rotated_scan.rangeYM(:) - fitted_line_vals_range(:,1);
    residual_intensity = rotated_scan.intensityYM(:) - fitted_line_vals_intensity(:,1);
%     size_residual = size(residual_range,2);
    if usegavin_func; %new one
        mse_range = mse_gp(residual_range(:));
    else
        mse_range = mse(residual_range(:));
    end

    if usegavin_func; %new one
        mse_intensity = mse_gp(residual_intensity(:));
    else
        mse_intensity = mse(residual_intensity(:));
    end
    
%     mse_size = size(mse_range,1);


%% Probability Based Classifier
%     load('Classification_Criteria.mat');

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
%     PClass.Confidence = lhoods.ratio/(1+lhoods.ratio);

    switch(material_to_class)
        case 1
%             PClass.Material = 'GSteel';
            guess = 1;
        case 2
%             PClass.Material = 'Alumin';
            guess = 2;
        case 3
%             PClass.Material = 'GMetal';
            guess = 3;
        case 4
%             PClass.Material = 'MSteel';  
            guess = 4;
        case 5
%             PClass.Material = 'Copper';
            guess = 5;
        case 6
%             PClass.Material = 'DPlyWd';
            guess = 6;
        case 7
%             PClass.Material = 'BCloth'; 
            guess = 7;
    end   

    % puts the result in an accessible place
    found_lines.classifier_output(i,1)= guess;

%% DEAL WITH NO CLASSIFIED SURFACES BY SPLITTING THEM AND RE-CLASSIFING
    if found_lines.classifier_output(i,1) == 4    
        if (found_lines.line_start_end_points_smoothed(i,2)-found_lines.line_start_end_points_smoothed(i,1))/2 > min_number_of_points_for_a_line
            found_lines.line_start_end_points_smoothed(end+1,1) = found_lines.line_start_end_points_smoothed(i,1);
            found_lines.line_start_end_points_smoothed(end,2) = floor(found_lines.line_start_end_points_smoothed(i,1)+((found_lines.line_start_end_points_smoothed(i,2)-found_lines.line_start_end_points_smoothed(i,1))/2));
            found_lines.line_start_end_points_smoothed(end+1,2) = found_lines.line_start_end_points_smoothed(i,2);
            found_lines.line_start_end_points_smoothed(end,1) = floor(found_lines.line_start_end_points_smoothed(i,2)-((found_lines.line_start_end_points_smoothed(i,2)-found_lines.line_start_end_points_smoothed(i,1))/2))+1;
            found_lines.line_start_end_points_smoothed(i,1) = -found_lines.line_start_end_points_smoothed(i,1);
            found_lines.line_start_end_points_smoothed(i,2) = -found_lines.line_start_end_points_smoothed(i,2);
            found_lines.found_lines_gradients(end+1,1) = found_lines.found_lines_gradients(i,1);
            found_lines.found_lines_gradients(end+1,1) = found_lines.found_lines_gradients(i,1);
       end
    end
end
    