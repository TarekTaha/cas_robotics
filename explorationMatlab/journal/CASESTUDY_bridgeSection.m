function result=CASESTUDY_bridgeSection
cartYoffset=0.11;
    color = [0.0625    0.5000    0.9375];
    % flange size
    flange = rectangularPrism;
    flange.width = 3.600;
    flange.breadth = 2*0.085 + 0.018;
    flange.height = 0.018;
    flange.color = color;
    % web size
    web = rectangularPrism;
    web.width = flange.width;
    web.breadth = 0.018;
    web.height = 0.500;
    web.color = color;
    % half ceiling size
    surface = rectangularPrism;
    surface.width = flange.width;
    surface.breadth = ((0.085+0.018)+1.222)/2;
    surface.height = 0.018;
    surface.color = color;
    robotOffset = flange.width - 1.430;

    % The origin is at the center of the bottom of one end
    % of one flange
    flange1 = shift(flange, [0, -flange.breadth / 2, 0]);
    web1 = shift(web, [0, -web.breadth / 2, flange.height]);
    flange2 = shift(flange, [0, -flange.breadth / 2, flange.height + web.height]);
    ibeam1 = compositeGlyph({flange1, web1, flange2});
    
    ceiling1 = shift(surface, [0, -flange.breadth / 2, ibeam1.height]);
    ceiling2 = shift(ceiling1, [0, surface.breadth, 0]);
    ibeam2 = shift(ibeam1, [0, 2 * surface.breadth - flange.breadth, 0]);
    
    section1 = compositeGlyph({ibeam1, ceiling1});
    % move origin to ceiling
    section1 = shift(section1, [0, 0, -ibeam1.height]);
    % rotate to achieve 8mm expansion at the bottom of the ibeam
    sagAngle = tan(0.004 / ibeam1.height);
    section1 = rotate(section1, xRotation(-sagAngle));
    % move origin to robot base
    section1 = shift(section1, [-robotOffset, -(surface.breadth - flange.breadth / 2)-cartYoffset, 1.498+0.5+3*surface.height - 0.540]);
    
    section2 = compositeGlyph({ibeam2, ceiling2});
    % move origin to ceiling above ibeam
    section2 = shift(section2, -[0, 2 * surface.breadth - flange.breadth, ibeam1.height]);
    % rotate to achieve 8mm expansion at the bootom of the ibeam
    section2 = rotate(section2, xRotation(sagAngle));
    % move origin to robot base
    section2 = shift(section2, [-robotOffset, surface.breadth - flange.breadth / 2, 1.498+0.5+3*surface.height - 0.540]);
    
    % combine the sections
    fullSection = compositeGlyph({section1, section2});
    % adjust for the slope of the robot platform
%     robotAngle = tan(0.085 / 1.300);
%     fullSection = rotate(fullSection, zRotation(pi)*yRotation(robotAngle));

    if 0 < nargout
        result = fullSection;
    else
        vertices = fullSection.vertices;
        faces = fullSection.faces;
        
        %plot out for ease of finding
        plot3(vertices(:,1),vertices(:,2),vertices(:,3),'r*')
        
        
%Add the midpeice between vertices        
        %back mid small plane
        faces(end+1,:)=[32,61,28,57];
        %front mid small plane
        faces(end+1,:)=[31,62,58,27];
        %top mid plane
        faces(end+1,:)=[32,61,62,31];
        %top mid plane
        faces(end+1,:)=[31,28,57,58];

%Add vertices for the girder in the middle
        numverts=size(vertices,1);
        %15 is front left wall top end % 16 is back left wall top end
        vertices(numverts+1,:)=(vertices(15,:)+vertices(16,:))/2;
        %45 is back right wall top end % 46 is front right wall top end
        vertices(numverts+2,:)=(vertices(45,:)+vertices(46,:))/2;
        %41 is back right wall bottome end %42 is front right wall bottom end 
        vertices(numverts+3,:)=(vertices(41,:)+vertices(42,:))/2 + [0,0,0.07];       
        
        %between mid roof and mid end
        vertices(numverts+4,:)=(vertices(numverts+3,:)+vertices(numverts+1,:))/2;
%         %57 is back mid top end % 58 is front mid top end
%         vertices(numverts+2,:)=(vertices(57,:)+vertices(58,:))/2;
        faces(end+1,:)=[numverts+1,numverts+2,numverts+3,numverts+4];
        %plot in new vertices for the middle girder in blue
        hold on;plot3(vertices(numverts+1:end,1),vertices(numverts+1:end,2),vertices(numverts+1:end,3),'b*')
        grid on;axis equal        
        
%Add extra vertices for the crossbeam in the back         
        numverts=size(vertices,1);
        %RIGHT crosbeam vert
        %41 is back right wall bottom end %42 is front right wall bottom end
        %so it is 7cm away from mid towards back
%test 6 vertices(numverts+1,:)=(vertices(41,:)+vertices(42,:))/2+[-0.4,0,0];       
%test10
        vertices(numverts+1,:)=(vertices(41,:)+vertices(42,:))/2+[-0.2,0,0];       
        %slightly back along x on right flange
        vertices(numverts+2,:)=vertices(numverts+1,:)-[0.07,0,0];       
        %slightly up (z) above front, on right flange
        vertices(numverts+3,:)=vertices(numverts+1,:)+[0,0,0.07];       
        %slightly up (z) above back, on right flange
        vertices(numverts+4,:)=vertices(numverts+2,:)+[0,0,0.07];     

        %LEFT crosbeam verts
        %12 is back left wall bottom end %11 is front left wall bottom end
        %so it is 7cm away from mid towards back
% test6  vertices(numverts+5,:)=(vertices(12,:)+vertices(11,:))/2 +[-0.4,0,0];       
%test10
        vertices(numverts+5,:)=(vertices(12,:)+vertices(11,:))/2 +[-0.2,0,0];       
        %slightly back along x on left flange
        vertices(numverts+6,:)=vertices(numverts+5,:)-[0.07,0,0];       
        %slightly up (z) above front, on right flange
        vertices(numverts+7,:)=vertices(numverts+5,:)+[0,0,0.07];       
        %slightly up (z) above back, on right flange
        vertices(numverts+8,:)=vertices(numverts+6,:)+[0,0,0.07];       

        %join up the crossbeam bottom
        faces(end+1,:)=[numverts+1,numverts+2,numverts+6,numverts+5];
        %join up the crossbeam front
        faces(end+1,:)=[numverts+1,numverts+3,numverts+7,numverts+5];
        %join up the crossbeam top
        faces(end+1,:)=[numverts+3,numverts+4,numverts+8,numverts+7];
        %join up the crossbeam back
        faces(end+1,:)=[numverts+2,numverts+4,numverts+8,numverts+6];
        
        %plot in new vertices for the middle girder in blue
        hold on;plot3(vertices(numverts+1:end,1),vertices(numverts+1:end,2),vertices(numverts+1:end,3),'g*')
        
        
        
        


        %END ADD EXTRA SHIT
        
        
        
         
        data.vertex.x = vertices(:,1);
        data.vertex.y = vertices(:,2);
        data.vertex.z = vertices(:,3);
        
        for i=1:size(faces,1)
            data.face.vertex_indices{i} = faces(i,:)-1;
        end
        plywrite(data,'CASESTUDY_bridgeSection.ply','ascii');
    end
    
    
    trisurf(cell2mat(data.face.vertex_indices)+1,data.vertex.x, data.vertex.y, data.vertex.z, 'FaceColor', 'None');
    
end

function r=xRotation(t)
	ct = cos(t);
	st = sin(t);
	r =[1	0	0
		0	ct	-st
		0	st	ct];
end
 
function r=yRotation(t)
	ct = cos(t);
	st = sin(t);
	r =[ct	0	-st
		0	1	0
		st	0	ct];
end

function r=zRotation(t)
	ct = cos(t);
	st = sin(t);
	r =[ct	-st  0
		st	ct  0
		0	0	1];
end
   