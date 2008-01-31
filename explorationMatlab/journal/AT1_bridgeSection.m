function result=AT1_bridgeSection
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
    robotOffset = flange.width - 1.440;

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
    section1 = shift(section1, [-robotOffset, -(surface.breadth - flange.breadth / 2)-cartYoffset, 1.5+0.5+3*surface.height - 0.540]);
    
    section2 = compositeGlyph({ibeam2, ceiling2});
    % move origin to ceiling above ibeam
    section2 = shift(section2, -[0, 2 * surface.breadth - flange.breadth, ibeam1.height]);
    % rotate to achieve 8mm expansion at the bootom of the ibeam
    section2 = rotate(section2, xRotation(sagAngle));
    % move origin to robot base
    section2 = shift(section2, [-robotOffset, surface.breadth - flange.breadth / 2, 1.5+0.5+3*surface.height - 0.540]);
    
    % combine the sections
    fullSection = compositeGlyph({section1, section2});
    % adjust for the slope of the robot platform
%     robotAngle = tan(0.085 / 1.300);
%     fullSection = rotate(fullSection, zRotation(pi)*yRotation(robotAngle));

    if 0 < nargout
        result = fullSection;
    else
        vertices = fullSection.vertices;
        data.vertex.x = vertices(:,1);
        data.vertex.y = vertices(:,2);
        data.vertex.z = vertices(:,3);
        faces = fullSection.faces;
        for i=1:size(faces,1)
            data.face.vertex_indices{i} = faces(i,:)-1;
        end
        plywrite(data,'AT1_bridgeSection.ply','ascii');
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
   