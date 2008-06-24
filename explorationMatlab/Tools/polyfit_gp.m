%% polyfit_gp
%
% *Description*: This file is for first and second order polyfitting and will be used inside the classifier

%% Function Call
%
% *Inputs:* 
%
% _xvals_ (1 * many double) These are the xpoints
%
% _yvals_ (1 * many double) These are the ypoints
%
% _order_ (int) only 1st and 2nd supported
%
% *Outputs:* 
%
% _polyvals_ (either 1*2 or 1*3 double) [x1 x0] or [x2 x1 x0]
%

function [polyvals]=polyfit_gp(xvals,yvals,order)

%% Input Checks
if nargin<3
  error('need to pass in correct num of vars')
end

xvals=xvals(:);
yvals=yvals(:);

V(:,order+1) = ones([size(xvals,1),1],'double');
for j = order:-1:1
   V(:,j) = xvals.*V(:,j+1);
end
[Q,R] = qr(V,0);
polyvals = [R\(Q'*yvals)]';       
