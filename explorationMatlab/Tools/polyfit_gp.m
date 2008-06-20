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
% _order_ (1) only 1st and second supported
%
% *Outputs:* 
%
% _x1_ (double) This is the first order
%
% _x0_ (double) This is the zero order
%

function [x1,x0]=polyfit_gp(xvals,yvals,order)

%% Input Checks
if nargin<3
  error('need to pass in correct num of vars')
end

if order>1
    error('not suported yet');
end

%% Functions
nrows=size(xvals,1);

% the mean to subtract
mean_to_sub=sum([xvals,yvals])/nrows;

% the data minus the mean of data
data_minusmean=[xvals-mean_to_sub(1),yvals-mean_to_sub(2)];

% better covariance than this ->% convergange_mat=cov(data_minusmean);
%get the convergance matrix of points and eigen values and vectors
convergange_mat = (data_minusmean' * data_minusmean) / (nrows-1); 

%get eigen values and vectors
[eigenvectors,eigenvalues]=eig(convergange_mat);
        
%make into a 3*1 matrix so we have each eigenvalue
eigenvalues=eigenvalues*[1;1];

%order the eigenvalues so we get the best fit on top
min2max_eig=sort(eigenvalues,'descend');

%determine data correlation (the higher the better
if min2max_eig(2)>0
    data_correlation=min2max_eig(1)/min2max_eig(2);
else
    data_correlation=[Inf];
end

%this meeans there is a 40* data correlation
if data_correlation>0
    fitXYvector=eigenvectors(:,(eigenvalues==min2max_eig(1))); 

    %determine m (AKA x1) and yintercept (AKA x0)
    x1=fitXYvector(2)/fitXYvector(1);
    x0=mean_to_sub(2)-x1*mean_to_sub(1);
% else
%     polyfit_line_parameters=polyfit(xvals,yvals,1);
%     x1=polyfit_line_parameters(2);
%     x0=polyfit_line_parameters(1);
% end

% close all
% plot(xvals,yvals,'r.');
% f = polyval([x1,x0],min(xvals):0.5:max(xvals));
% hold on;
% plot(min(xvals):0.5:max(xvals),f,'b')
% 
% f = line_parameters(1)*[min(xvals):0.5:max(xvals)]+line_parameters(2);
% hold on;
% plot(min(xvals):0.5:max(xvals),f,'g')
