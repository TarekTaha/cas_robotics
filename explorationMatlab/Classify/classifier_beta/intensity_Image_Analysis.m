%% intensity_Image_Analysis
% *Description:* 
% Show an intensoty image and analsyis that image

function intensity_Image_Analysis(positiondata,source,vertices,show_intensity_image,process_intensity_image)

if nargin<5
    process_intensity_image=true;
    if nargin<4
        show_intensity_image=true;
    end
end

rays = length(positiondata.intensity.scan{1});
scans = length(positiondata.intensity.scan);
intense_im = reshape([positiondata.intensity.scan{:}], rays, scans);
autogain_im = reshape([positiondata.autogain.scan{:}], rays, scans);
%normalise the intensity
norm_intense_im=(intense_im./autogain_im);
%some autogain values are 0 so the result is inf, we have to remove these
%and make them the same as the maximum value
norm_intense_im(norm_intense_im==inf)=max(max(norm_intense_im(norm_intense_im<inf)));
%in some cases both are 0 which results in a NaN so search for these and
%replace with the lowest value
norm_intense_im(isnan(norm_intense_im))=0;
%now normalise between 0 and 1
norm_intense_im=norm_intense_im/max(max(norm_intense_im));
%turn to the side
norm_intense_im=norm_intense_im';

if show_intensity_image
    imshow(norm_intense_im);
end

if process_intensity_image
    figure    
    subplot(2,2,1);
    Medfiltdata=medfilt2(norm_intense_im,[5 5]);
    imshow(Medfiltdata )
    title('Median filtered data 5by5');

    subplot(2,2,2);
    K = imadjust(Medfiltdata,[156/255 188/255],[]);
    imshow(K)
    title('Intensity change on median filtered data full band stretched for original 156/255->188/255');

   
    subplot(2,2,3)
    BW2_K=edge(K,'canny');
    imshow(BW2_K)
    title('Edge detection (CANNY) on the intensity image change');
    
    %image on the to BW image and the intensity to BW
    subplot(2,2,4)
    BW2_roi = roicolor(K,128/255,255/255);
    BW2_roi = edge(uint8(BW2_roi),'canny');
    imshow(BW2_roi);   
    title('Intensity of interest (128->255) then edge detection (CANNY)');
    
    %note how this is around the other way 2,1
    [edgepixels(:,2),edgepixels(:,1)]=find(BW2_roi);
    touse=round(source(:,5:6));
    [nothing,edgePixel_indexofverts]=intersect(touse,edgepixels,'rows');
    plot3(vertices(edgePixel_indexofverts,1),vertices(edgePixel_indexofverts,2),vertices(edgePixel_indexofverts,3),'b*')
    hold on;
    plot3(vertices(:,1),vertices(:,2),vertices(:,3),'color','r','marker','.','markersize',4,'linestyle','none');
    axis equal
    
end


