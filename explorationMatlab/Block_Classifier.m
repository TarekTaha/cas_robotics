function Block_Classifier(PointData, IntensityData, RangeData) % this one uses edges from IntensityData Image

%% Flags
do_smoothing = 1; %Set to 1 to do median filtering to smooth the classification result
draw_image = 1; %Set to 1 to draw an 2D image of the PC
draw_PC = 0; %Set to 1 to draw the PC
display_ID_edgeim = 0; %displays an image of the intensity data and an image of teh edges found in the intensity image
draw_image_with_edge = 1; %displays the images with the found edges marked on them
dont_run_classifier = 0; %doesn't classify data -used when ClassifiedData already exists
replace_hoks_false_intensity_data = 1; %This  replaces the hokuyo's false data with the intensity reading from the ray next to the non-returned  ray but on the same scan
find_surfaces_in_PC = 1; % This finds surface in the PC - used for AOI scaling in the classifier

classiy_scan_range_start = 1;
classiy_scan_range_end = size(PointData,1);

colordef black;

%% Find surfaces in the point cloud
if find_surfaces_in_PC == 1
    [PointsPlaneData, surface_plane_coefs] = surface_making_simple(PointData,0.2);
end

%% The hokuyo uses the last data for a ray if no return is recieved - This
% causes a problem with edge detection. This cell replaces the hokuyo's
% false data with the intensity reading from the ray next to the non-returned 
% ray but on the same scan - so in other waords, more false data, but more
% realistic false data
if replace_hoks_false_intensity_data == 1;
    IntensityDataFlaseDataFixed = IntensityData;
    last_known_good_data(size(IntensityDataFlaseDataFixed,2)) = 0;
    last_known_good_data(:) = IntensityDataFlaseDataFixed(1,1);

    for i = 2:size(IntensityDataFlaseDataFixed,2);
        for j = 2:size(IntensityDataFlaseDataFixed,1);
            if IntensityDataFlaseDataFixed(j,i) == last_known_good_data(i)
                IntensityDataFlaseDataFixed(j,i) = IntensityDataFlaseDataFixed(j,i-1);
            else 
                last_known_good_data(i) = IntensityDataFlaseDataFixed(j,i);
            end
        end
    end
end

if replace_hoks_false_intensity_data == 1;
    IntensityDatatoUse = IntensityDataFlaseDataFixed;
else
    IntensityDatatoUse = IntensityData;
end

%clean up
clear last_known_good_data;
clear IntensityDataFlaseDataFixed;
clear i;
clear j;

%% Edge detecting in IntensityData image
%edge detection method to use
% edge_finding_method_to_use = 'sobel';
% edge_finding_method_to_use = 'prewitt';
 edge_finding_method_to_use = 'roberts';
% edge_finding_method_to_use = 'log';
% edge_finding_method_to_use = 'zerocross';
% edge_finding_method_to_use = 'canny';

InData4Image = IntensityDatatoUse/max(max(IntensityDatatoUse));
image_being_used = InData4Image;

%finds edges in the image
Iedges = edge(image_being_used, edge_finding_method_to_use);

%displays images
if display_ID_edgeim == 1
    figure, imshow(image_being_used);
    figure, imshow(Iedges);
end

%% Init's classification result matrix and does classify
if dont_run_classifier == 0
    ClassifiedData = IntensityDatatoUse; % creats a variable of the correct size - one classifcation per point
    ClassifiedData(:,:) = 4; % sets all points classification to 4 - 'don't know'

    %% Sends data to the classifier one scan at a time and creates a matrix of classification results - one for each point
    for i = classiy_scan_range_start:classiy_scan_range_end  
    if mod(i,20) == 0 % puts something in debug so I know its working
        i
    end
        try % I use try as sometimes the classifier crashes for an unknown reason
            if find_surfaces_in_PC == 0
                found_lines = Classifier(PointData, IntensityDatatoUse, RangeData, i, Iedges); % this one uses edges from IntensityData Image
            else
                found_lines = Classifier(PointData, IntensityDatatoUse, RangeData, i, Iedges, PointsPlaneData, surface_plane_coefs); % this one uses edges from IntensityData Image
            end
        end
        % This creats the classfier output matrix. The matrix is in the same
        % format as PointData, IntensityData, etc
        number_of_lines = size(found_lines.line_start_end_points_smoothed,1); 
        for j = 1:number_of_lines
            if found_lines.line_start_end_points_smoothed(j,1) > 0
                number_of_points_classified_by_this_line = found_lines.line_start_end_points_smoothed(j,2)-found_lines.line_start_end_points_smoothed(j,1)+1;
                for k = 1:number_of_points_classified_by_this_line
                    ClassifiedData(i,(found_lines.line_start_end_points_smoothed(j,1)+k-1)) = found_lines.classifier_output(j);
                end 
            end
        end
    end
end

%% Put edges (found from intensity data) on the Classified data 
for i = 1:size(Iedges,1)
    for j = 1:size(Iedges,2)
        if Iedges(i,j) == 1
            ClassifiedDatawEdges(i,j) = 5;
        else
            ClassifiedDatawEdges(i,j) = ClassifiedData(i,j);
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
    figure();
    view(-90,-90);
    hold on;
    for i = 1:size(data_to_display,1)
        for j = 1:size(data_to_display,2)
            %for user output colouring
            switch (data_to_display(i,j))
                case 0
                    %output = 'void';
                    output_color = [0 0 0]; % BLACK
                case 1
                    %output = 'Grey Metal ';
                    output_color = [.35 .35 .35]; % GREY
                case 2
                    %output = 'Shiny Metal';
                    output_color = [.8 .8 .8]; % Silver
                case 3
                    %output = 'Cloth/Wood'; % OR RED OR WHITE --- JUST CLOTH or WOOD!!!
                    output_color = [1 0 0]; % RED
                case 4
                    %output = 'Do not know';
                    output_color = [0 0 0]; % BLACK 
                case 5
                    %output = 'Edge';
                    output_color = [.7 .9 .0]; % GREEN 
            end 
            plot3(PointData(i,j,1),PointData(i,j,2),PointData(i,j,3),'color', output_color);
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
    colmap = [[.35 .35 .35]; [.8 .8 .8]; [1 0 0]; [0 0 0]; [.7 .9 .0]];
    colormap(colmap)
    image(data_to_display);
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
                ClassifiedDataSmoothedwEdges(i,j) = 5;
            else
                ClassifiedDataSmoothedwEdges(i,j) = ClassifiedDataSmoothed(i,j);
            end
        end
    end
   
    %---- Cut and paste from above ----------------------------------------
        % Point Cloud
        if draw_PC == 1
            %Coloured (by classification) point cloud
            if draw_image_with_edge == 1
                data_to_display = ClassifiedDataSmoothedwEdges;
            else
                data_to_display = ClassifiedDataSmoothed;
            end
            figure();
            view(-90,-90);
            hold on;
            for i = 1:size(data_to_display,1)
                for j = 1:size(data_to_display,2)
                    %for user output colouring
                    switch (data_to_display(i,j))
                        case 0
                            %output = 'void';
                            output_color = [0 0 0]; % BLACK
                        case 1
                            %output = 'Grey Metal ';
                            output_color = [.35 .35 .35]; % GREY
                        case 2
                            %output = 'Shiny Metal';
                            output_color = [.8 .8 .8]; % Silver
                        case 3
                            %output = 'Cloth/Wood'; % OR RED OR WHITE --- JUST CLOTH or WOOD!!!
                            output_color = [1 0 0]; % RED
                        case 4
                            %output = 'Do not know';
                            output_color = [0 0 0]; % BLACK 
                        case 5
                            %output = 'Edge';
                            output_color = [.7 .9 .0]; % GREEN 
                    end 
                    plot3(PointData(i,j,1),PointData(i,j,2),PointData(i,j,3),'color', output_color);
                end 
            end
            drawnow;
        end

        %% Image of PC
        if draw_image == 1
            if draw_image_with_edge ==1
                data_to_display = ClassifiedDataSmoothedwEdges;
            else
                data_to_display = ClassifiedDataSmoothedData;
            end
            figure();
            colmap = [[.35 .35 .35]; [.8 .8 .8]; [1 0 0]; [0 0 0]; [.7 .9 .0]];
            colormap(colmap)
            image(data_to_display);
        end
    %----------------------------------------------------------------------
end


%% Find Surfaces in the point cloud - Curtiousy of Gav'

        % This function goes through each point and register it to one or more planes
        function [PointsPlaneData, surface_plane_coefs] = surface_making_simple(PointData,mew);

        %make into a 3 by many matrix of points 
        % (it would be better if a 1* matrix could be passed to this function instead
        scan_data=zeros([size(PointData,1)*size(PointData,2),3]);
        for i=1:size(PointData,1);
            scan_data((i-1)*size(PointData,2)+1 : i*size(PointData,2) , :)=...
                [[PointData(i,:,1)]',[PointData(i,:,2)]',[PointData(i,:,3)]'];                  
        end

        %other variables
        plane_num=1;
        point_cloud_reg=zeros([size(scan_data,1),1]);
        min_num_pts_in_plane=8;
        eigen_value_thresh=2;

        %breakinto cubes
        start_time=clock;
        cube_size=mew;
        [point_in_cube]=break_into_cubes_internal(scan_data,cube_size);
        voxelate_time=etime(clock,start_time);

        %Go through and register the points and make planes
        for current_point=1:size(scan_data,1)
            %if not registered to a plane try and resister it or start its own
            if point_cloud_reg(current_point)==0
                %get the distance from all points to current point
                distance_between=distance_between_all_points_internal(scan_data,current_point,mew,point_in_cube);

                 if length(distance_between.closest_indices)>min_num_pts_in_plane        
                     %make the point group zero mean
                     mean_to_sub=sum(scan_data(distance_between.closest_indices,:))/...
                                     length(distance_between.closest_indices);
                     scan_data_minusmean=[scan_data(distance_between.closest_indices,1)-mean_to_sub(1),...
                                          scan_data(distance_between.closest_indices,2)-mean_to_sub(2),...
                                          scan_data(distance_between.closest_indices,3)-mean_to_sub(3)];    
                    %get the convergance matrix of points and eigen values and vectors
                    convergange_mat=cov(scan_data_minusmean);
                    [eigenvectors,eigenvalues]=eig(convergange_mat);
                    eigenvalues=eigenvalues*[1;1;1];

                    %sort 3 eigenvalues so as to get data correlation
                    min2max_eig=sort(eigenvalues,'ascend');
                    if min2max_eig(1)>0
                        data_correlation=min2max_eig(2)/min2max_eig(1);
                    else
                        data_correlation=[Inf];
                    end

                    %finds which has least correlation =normal and the greatest
                    %is on the plane
                    norm=eigenvectors(:,find(eigenvalues==min2max_eig(1)));                
                    temp_plane_equ=[norm(1),norm(2),norm(3),...
                                  -(norm(1)*scan_data(current_point,1)+norm(2)*scan_data(current_point,2)+norm(3)*scan_data(current_point,3))];

                    if (dis_bet_plane_n_pnt_internal(temp_plane_equ,scan_data(current_point,:))<mew && data_correlation>eigen_value_thresh)
                        dis_to_plane=dis_bet_plane_n_pnt_internal(temp_plane_equ,scan_data(distance_between.closest_indices,:));
                        plane(plane_num).home_point=scan_data(current_point,:);
                        plane(plane_num).equ=temp_plane_equ;
                        plane(plane_num).normal_by_eigenval=norm;
                        plane(plane_num).points=distance_between.closest_indices(find(dis_to_plane<mew));
                        point_cloud_reg(plane(plane_num).points)=1; 
                        plane_num=plane_num+1;                    
                    end            
                 end %end the if to see if there are at least 3 points within mew
            end %end the if unregistered current point         
        end %end the for loop for every point

        % prepare output data
        surface_plane_coefs=zeros(plane_num-1,4);
        for i = 1:plane_num-1
            surface_plane_coefs(i,1:4) = plane(1,i).equ;
        end

        %assign points to a plane they belong to, remember they could belong to
        %more than one plane and this will only assign them to one of these
        point_belongs_to=zeros([size(scan_data,1),1]);
        for i=1:size(plane,2)
            point_belongs_to(plane(i).points)=i;
        end

        % puts back in original form
        PointsPlaneData=zeros([size(PointData,1),size(PointData,2)]);
        for i=1:size(PointsPlaneData,1)
            PointsPlaneData(i,1:size(PointsPlaneData,2))=point_belongs_to((i-1)*size(PointsPlaneData,2)+1:i*size(PointsPlaneData,2))';
        end
        %shows the intensity image
        % imshow(PointsPlaneData/max(max(PointsPlaneData)));

        % Printing out details - this is not needed
        % Print out the time taken and the points covered
        perc_of_pnts_reg=100*(1-size(find(point_cloud_reg==0),1)/size(point_cloud_reg,1));
        calc_time=etime(clock,start_time);
        %determine memory use
        temp=whos('scan_data','bytes');
        memB4=temp.bytes;
        temp1=plane(1).home_point;temp2=whos('temp1');
        temp3=plane(1).equ;temp4=whos('temp3');
        memafta=(temp2.bytes+temp4.bytes)*length(plane);

        %%%%%%%%
        % Display Algorithm Statistics
        % disp(strcat('From a point cloud with:_',num2str(size(scan_data,1)),'_points'));
        % disp(strcat('_Dividing into cubes took:_', num2str(voxelate_time),'_We used mew=_', num2str(mew),'_Min no. pts in plane=_',num2str(min_num_pts_in_plane),'_. The total time was:_',num2str(calc_time),'_sec.')); 
        % disp(strcat('We made:_',num2str(size(plane,2)),'_planes and registered:_',num2str(perc_of_pnts_reg),'_% of pnts'));
        % disp(strcat('This reduced the data size from:_',num2str(memB4),'_to:_',num2str(memafta)));

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % ADDITIONAL FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Function:break_into_cubes_internal
        %breaks into cubes so that the searching is quicker
        function [point_in_cube]=break_into_cubes_internal(scan_data,cube_size)

        min_max=[min(scan_data(:,1)),max(scan_data(:,1));...
                 min(scan_data(:,2)),max(scan_data(:,2));...
                 min(scan_data(:,3)),max(scan_data(:,3))];
        shifed_points=[scan_data(:,1)-min_max(1,1),...
                       scan_data(:,2)-min_max(2,1),...
                       scan_data(:,3)-min_max(3,1)];
        %this fixes up the cubes so they start at 1 and go through to end
        in_which_cube=floor(shifed_points/cube_size)+1;
        max_values=max(in_which_cube);

        point_in_cube=zeros([size(scan_data)]);

        for i=1:max_values(1)
            point_in_cube(find(in_which_cube(:,1)==i),1)=i;
        end
        for j=1:max_values(2)
            point_in_cube(find(in_which_cube(:,2)==j),2)=j;
        end
        for k=1:max_values(3)                    
            point_in_cube(find(in_which_cube(:,3)==k),3)=k;
        end


        % distance_between_all_points_internal
        %this will give the distance between all points
        function distance_between=distance_between_all_points_internal(scan_data,current_point,mew,point_in_cube)

        hc=point_in_cube(current_point,:);

        a=            find(point_in_cube(:,1)==hc(1)-1 | point_in_cube(:,1)==hc(1) | point_in_cube(:,1)==hc(1)+1);
        b=            a(find(point_in_cube(a,2)==hc(2)-1 | point_in_cube(a,2)==hc(2) | point_in_cube(a,2)==hc(2)+1));
        close_pts_ind=b(find(point_in_cube(b,3)==hc(3)-1 | point_in_cube(b,3)==hc(3) | point_in_cube(b,3)==hc(3)+1));


        distance_between.all_values=sqrt((scan_data(current_point,1)-scan_data(close_pts_ind,1)).^2+...
                                         (scan_data(current_point,2)-scan_data(close_pts_ind,2)).^2+...
                                         (scan_data(current_point,3)-scan_data(close_pts_ind,3)).^2);
        distance_between.indices=close_pts_ind;
        temp=find(distance_between.all_values<mew);
        [distance_between.closest_values,temp_order]=sort(distance_between.all_values(temp),'ascend');
        distance_between.closest_indices=distance_between.indices(temp(temp_order));


        % dis_bet_plane_n_pnt_internal
        function dis_to_plane=dis_bet_plane_n_pnt_internal(plane_equ,pnt)
        %dis_to_plane=zeros([1,size(pnt,1)]);
        if size(pnt,1)>1
                dis_to_plane=abs((plane_equ(1)*pnt(:,1)+...
                                               plane_equ(2)*pnt(:,2)+...
                                               plane_equ(3)*pnt(:,3)+plane_equ(4))./...
                                               sqrt(plane_equ(1)^2+plane_equ(2)^2+plane_equ(3)^2));

        else
            dis_to_plane=abs((plane_equ(1)*pnt(1)+plane_equ(2)*pnt(2)+plane_equ(3)*pnt(3)+plane_equ(4))/...
                     sqrt(plane_equ(1)^2+plane_equ(2)^2+plane_equ(3)^2));
        end
