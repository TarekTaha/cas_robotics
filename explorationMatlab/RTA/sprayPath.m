% load tempplane
% plane = tempplane;
% mew = 0.04;
% plane_sphere_threshold=0.001;

function sprayPath(plane,mew,interval,color)
    if exist('mew') == 0, mew = 0.04; end
    if exist('interval') == 0, interval = 0.5; end
    if exist('color') == 0, color = 'r'; end
    %if exist('fill') == 0, fill = 1; end %0=Don't fill, 1=fill
    %start with a normal sphere described by 20 points
    plane_sphere_threshold=0.0005;
    [base_x,base_y,base_z]=sphere(50);
    %make the radium mew
    base_x=base_x*mew;
    base_y=base_y*mew;
    base_z=base_z*mew;
    %for each plane
    plot_handles=zeros([length(plane),1]);
    hold on;
    grid
    i = 1;
    for i=1:length(plane)
        %Set a sphere to have a center at home point
        locuspts_x=base_x+plane(i).home_point(1);
        locuspts_y=base_y+plane(i).home_point(2);
        locuspts_z=base_z+plane(i).home_point(3);
        %find out which points on the sphere correspond to where the plane
        %through the home point
        pts_to_check=plane(i).equ(1)*locuspts_x+...
                     plane(i).equ(2)*locuspts_y+...
                     plane(i).equ(3)*locuspts_z+...
                     plane(i).equ(4);
         %points that are within 0.1 are considered on the plane
         pts_to_plot=find(abs(pts_to_check)<plane_sphere_threshold);
         ez = [locuspts_x(pts_to_plot(1)),locuspts_y(pts_to_plot(1)),locuspts_z(pts_to_plot(1))];
         ey = [locuspts_x(pts_to_plot(2)),locuspts_y(pts_to_plot(2)),locuspts_z(pts_to_plot(2))];
         [x,y,z] = circlePoints( plane(i).home_point, ez, ey, mew, 50 ); 
         %if fill == 0 
             %plot3=(x,y,z,color)
         %end
         %if fill > 0 
             plot3=fill3(x,y,z,color); 
         %end
         pause(interval);
    end
end

function [x,y,z] = circlePoints( rc, r1, r2, R, N ) 
    x = [];
    y = [];
    z = []; 

    i = sqrt( -1 ); 

    ex = [1 0 0];
    ey = [0 1 0];
    ez = [0 0 1];

    if exist('N') == 0, N = 50; end
    if exist('R') == 0, R =  1; end

    if length( rc ) ~= 3, disp('Circle'' center should be a 3D vector!'), return, end 
    if length( r1 ) ~= 3, disp('r1 should be a 3D vector!'), return, end 
    if length( r2 ) ~= 3, disp('r2 should be a 3D vector!'), return, end

    R1 = r1 - rc;
    R2 = r2 - rc; 

    tolerance = 1 - abs( sum( R1.*R2 )/( norm( R1 )*norm( R2 ) ) );

    if tolerance < 1e-10, disp('r1 and r2 are colinear within the functions'' tolerance!'), return, end

    R1_times_R2 = vector_product( R1 , R2 ); 
    R3          = vector_product( R1, R1_times_R2 );  

    Ex = R1/norm( R1 );
    Ey = R3/norm( R3 );
    Ez = R1_times_R2/norm( R1_times_R2 );

    delta_theta = 2*pi/N;

    theta = [0:delta_theta:2*pi];

    z = exp( i*theta );

    xc = R*real( z );
    yc = R*imag( z );
    zc = zeros([1 N+1]); clear z

    R(1,1) = sum( Ex.*ex ); R(1,2) = sum( Ex.*ey ); R(1,3) = sum( Ex.*ez ); 
    R(2,1) = sum( Ey.*ex ); R(2,2) = sum( Ey.*ey ); R(2,3) = sum( Ey.*ez ); 
    R(3,1) = sum( Ez.*ex ); R(3,2) = sum( Ez.*ey ); R(3,3) = sum( Ez.*ez );

    for n = 1:N+1

        r(:,n) = R*[xc(n) yc(n) zc(n)]';

    end 

    x = r(1,:) + rc(1);
    y = r(2,:) + rc(2);
    z = r(3,:) + rc(3);

    function a_times_b = vector_product( a , b ) ; 

    a_times_b = [ ] ;

    a_times_b(1) = a(2)*b(3) - a(3)*b(2);  
    a_times_b(2) = a(3)*b(1) - a(1)*b(3); 
    a_times_b(3) = a(1)*b(2) - a(2)*b(1); 
    end    
end