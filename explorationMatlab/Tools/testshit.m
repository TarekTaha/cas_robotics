

function testshit()
%function
pvals_prior=rand([1,3]);
x=[round(rand()*20):50]';
y=pvals_prior(1)*x.^2+pvals_prior(2)*x+pvals_prior(3);
% y=x.^2/2;

%now we try and esitmate pvals_prior
numitts=1000;
profile clear; profile on; 
func_to_test(numitts,x,y)
profile off; profile viewer


function func_to_test(numitts,xvals,yvals)

for i=1:numitts
    pvals1=polyfit_gp(xvals,yvals,2);
    pvals2=polyfit(xvals,yvals,2);
    if abs(pvals1(3)-pvals2(3))>0.001 || abs(pvals1(2)-pvals2(2))>0.001 ||abs(pvals1(1)-pvals2(1))>0.001;
        display('problem');
    end
pvals_prior=rand([1,3]);
xvals=[round(rand()*40):50]';
yvals=pvals_prior(1)*xvals.^2+pvals_prior(2)*xvals+pvals_prior(3);
    
end
% for i=1:numitts
% %     fprintf('Something or other');
%     a=putInVoxels_matlab(points,0.1);
% end