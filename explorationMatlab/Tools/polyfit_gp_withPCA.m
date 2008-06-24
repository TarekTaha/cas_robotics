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
% %this is doing first order like polyfit way
% polyfitway=0;
% 
% %% If second order
% if polyfitway || order>1   
    
    %Core of polyfit
    V(:,order+1) = ones([size(xvals,1),1],'double');
    for j = order:-1:1
       V(:,j) = xvals.*V(:,j+1);
    end
    [Q,R] = qr(V,0);
    polyvals = [R\(Q'*yvals)]';       
% else
%     nrows=size(xvals,1);
% 
%     % the mean to subtract
%     mean_to_sub=sum([xvals,yvals])/nrows;
% 
%     % the data minus the mean of data
%     data_minusmean=[xvals-mean_to_sub(1),yvals-mean_to_sub(2)];
% 
%     % better covariance than this ->% convergange_mat=cov(data_minusmean);
%     %get the convergance matrix of points and eigen values and vectors
%     convergange_mat = (data_minusmean' * data_minusmean) / (nrows-1); 
% 
%     %get eigen values and vectors
%     [eigenvectors,eigenvalues]=eig(convergange_mat);
% 
%     %make into a 3*1 matrix so we have each eigenvalue
%     eigenvalues=eigenvalues*[1;1];
% 
%     %order the eigenvalues so we get the best fit on top
%     max2min_eig=sort(eigenvalues,'descend');
% 
%     %this meeans there is a 40* data correlation
%     fitXYvector=eigenvectors(:,(eigenvalues==max2min_eig(1))); 
% 
%     %determine m (AKA x1) and yintercept (AKA x0)
%     x1=fitXYvector(2)/fitXYvector(1);
%     x0=mean_to_sub(2)-x1*mean_to_sub(1);
% 
%     %value to return
%     polyvals=[x1,x0];
% end