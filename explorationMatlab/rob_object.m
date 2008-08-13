%% rob_object
%
% *Description:*  this sets up the robot object. The most important bit is the 
% length of the links and the type of joints etc. then rest of it is for
% the prisms that describe the links and sections of the robot. Nothing
% needs to exist prior to this function, it is standalone

%% Function Call
% 
% *Inputs:* NULL
%
% *Returns:* NULL

function r=rob_object()

%% Most Important Robot setup
% For each link specify
%alpha  ai ai 1     link twist angle (this affects the next section 
%A      Ai Ai 1     link length
%theta  qi qi       link rotation angle
%D      Di Di       link offset distance
%sigma  si si       joint type; 0 for revolute "standard", non-zero for prismatic
%offset sets the starting angle possition, needs to be considered for ALL future links

%         [ alpha      A       theta   D       sigma   offset],   CONVENTION   %absolute   relative to previous link includes offset

l1 = link([-1.5708,0.18,0,0.475,0,0],'standard');
l2 = link([0,0.385,0,0,0,-pi/2],'standard');
l3 = link([1.5708,-0.1,0,0,0,pi/2],'standard');
l4 = link([-1.5708,0,0,0.445,0,0],'standard');
l5 = link([1.5708,0,0,0,0,0],'standard');
l6 = link([0,0,0,0.084,0,0],'standard');

%% Define the link limits
l1.qlim = [deg2rad(-170),deg2rad(170)];
l2.qlim = [deg2rad(-90),deg2rad(135)];
l3.qlim = [deg2rad(-80),deg2rad(165)];
l4.qlim = [deg2rad(-185),deg2rad(185)];
l5.qlim = [deg2rad(-120),deg2rad(120)];
l6.qlim = [deg2rad(-360),deg2rad(360)];

% %% Now setup the glyphs that go around the links
% glyph1 = rectangularPrism(0.421,0.185,0.305);
% glyph2 = rectangularPrism(0.57,0.195,0.146);
% glyph3 = rectangularPrism(0.116,0.116,0.3579);
% glyph4 = rectangularPrism(0.116,0.233,0.15);
% glyph5 = rectangularPrism(0.114,0.13,0.084);
% glyph6 = rectangularPrism(0.082,0.082,0.006);
% 
% %% Then shift glyphs
% glyph1 = shift(glyph1, [-0.18-0.15,0,-0.155]);
% glyph2 = shift(glyph2, [-0.485,-0.0825,0.058]);
% glyph3 = shift(glyph3, [-0.058,-0.058,-0.14591]);
% glyph4 = shift(glyph4, [-0.058,0,-0.075]);
% glyph5 = shift(glyph5, [-0.057,-0.065,0]);
% glyph6 = shift(glyph6, [-0.041,-0.041,0]);
% 
% %% Then change the colours for the glyphs 
% glyph1.color = [0           0     0.50196];
% glyph2.color = [1  0  0];
% glyph3.color = [0           0     0.50196];
% glyph4.color = [0     0.50196           0];
% glyph5.color = [0  0  0];
% glyph6.color = [1     0.50196     0.25098];
% % glyph1.color = [0.50196];
% glyph2.color = [1];
% glyph3.color = [0.50196];
% glyph4.color = [0.50196];
% glyph5.color = [1];
% glyph6.color = [0.25098];

% %% Make the base glyph
% baseMountingGlyph  = rectangularPrism(0.305,0.305,0.29);
% baseCablesGlyph  = rectangularPrism(0.073,0.192,0.18);
% 
% baseMountingGlyph  = shift(baseMountingGlyph,[-0.15,-0.155,0]);
% baseCablesGlyph  = shift(baseCablesGlyph,  [-0.15-0.073,-0.096,0]);
% 
% % baseMountingGlyph.color = [0,0,0.50196];
% % baseCablesGlyph.color = [0,0,0.50196];
% baseMountingGlyph.color = [0.50196];
% baseCablesGlyph.color = [0.50196];
% 
% %% Specify the glyphs for each of the links
% l1.glyph = glyph1;
% lnk{1}=l1;
% 
% l2.glyph = glyph2;
% lnk{2}=l2;
% 
% l3.glyph = glyph3;
% lnk{3}=l3;
% 
% l4.glyph = glyph4;
% lnk{4}=l4;
% 
% l5.glyph = glyph5;
% lnk{5}=l5;
% 
% l6.glyph = glyph6;
% lnk{6}=l6;
% 
% r=robot(lnk);
% % compositeGlyph sticks together different Rect prisms
% r.baseGlyph = compositeGlyph({baseMountingGlyph, baseCablesGlyph});
% Give the robot a name
r.name = 'VMDArm';
% Default: Use glyphs to draw robot, dont display the name
r.plotopt = {'nojoints','noshadow', 'nowrist','noname'};