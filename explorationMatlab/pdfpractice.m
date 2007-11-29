function pdfpractice();

close all;
clear all;

% vals={'beta' 
% 'bino' 
% 'chi2' 
% 'exp' 
% 'ev' 
% 'f' 
% 'gam' 
% 'gev' 
% 'gp' 
% 'geo' 
% 'hyge'
% 'logn'
% 'nbin' 
% 'ncf'
% 'nct' 
% 'ncx2' 
% 'norm' 
% 'poiss' 
% 'rayl' 
% 't' 
% 'unif' 
% 'unid' 
% 'wbl' };


% for the normal it is X, mean 
X=-10:0.1:10;
mew=0;
sig=4;


Y = pdf('norm',X,mew,sig);
plot(X,Y)

for Xval=-10:0.05:10
    try delete(temp);
    end
    
    Yval=(mod(Xval,2)+1)*(1/(sig*sqrt(2*pi))*exp(-(Xval-mew)^2/(2*(sig)^2)));
    hold on;
    temp=plot(Xval,Yval,'r*');
    pause(0.1)
end

