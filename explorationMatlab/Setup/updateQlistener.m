
function updateQlistener(varargin)
  global Q
  if strcmpi(varargin{end}, 'JointState')
  % varargin{3} is a 1x6 matrix in degrees
  Q = deg2rad(varargin{3});
  end
end