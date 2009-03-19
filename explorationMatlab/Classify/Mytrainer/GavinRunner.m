% [PolCoefs] = CoEfsPlotter();
% [HParas] = Histo(PolCoefs);

load Classification_Criteria.mat
%materials: 1=Painted metal, 2=wood, 3=plastic
% materialnum=1;
% load material1-paintedmetal.mat
% % 
% materialnum=2;
% load material2-wood.mat %save('material2-wood.mat','mse_range_save','mse_intensity_save','polyfit_coefs_range_save','polyfit_coefs_intensity_save');
%  
materialnum=3;
load material3-plastic.mat

% clear global mse_range_save mse_intensity_save polyfit_coefs_range_save polyfit_coefs_intensity_save ;
global mse_range_save mse_intensity_save polyfit_coefs_range_save polyfit_coefs_intensity_save ;

% starting guess, needs to be manually tunned
if materialnum==1
  mse_range_thresh=0.8;
  mse_intensity_thresh=2500;
  polyfit_coefs_range_thresh_max=7e-007;
  polyfit_coefs_range_thresh_min=-7e-007;
  polyfit_coefs_intensity_thresh_max=0.0007;
  polyfit_coefs_intensity_thresh_min=-0.005;
elseif materialnum==2
  mse_range_thresh=6;
  mse_intensity_thresh=700;
  polyfit_coefs_range_thresh_max=1e-004;
  polyfit_coefs_range_thresh_min=-1e-004;
  polyfit_coefs_intensity_thresh_max=0.007;
  polyfit_coefs_intensity_thresh_min=-0.005;
elseif materialnum==3
  mse_range_thresh=2.5;
  mse_intensity_thresh=1300;
  polyfit_coefs_range_thresh_max=7e-008;
  polyfit_coefs_range_thresh_min=-6e-008;
  polyfit_coefs_intensity_thresh_max=0.0005;
  polyfit_coefs_intensity_thresh_min=-0.005;  
end
%   mse_range_thresh=inf;
%   mse_intensity_thresh=inf;
%   polyfit_coefs_range_thresh_max=inf;
%   polyfit_coefs_range_thresh_min=-inf;
%   polyfit_coefs_intensity_thresh_max=inf;
%   polyfit_coefs_intensity_thresh_min=-inf;

%% 
figure(1);hist(mse_range_save(mse_range_save>eps& mse_range_save<mse_range_thresh));
[HParas.RMSE.mean(materialnum),HParas.RMSE.std(materialnum)]=normfit(mse_range_save(mse_range_save>eps& mse_range_save<mse_range_thresh));

%%
figure(2);hist(mse_intensity_save(mse_intensity_save>eps& mse_intensity_save<mse_intensity_thresh))
[HParas.IMSE.mean(materialnum),HParas.IMSE.std(materialnum)]=normfit(mse_intensity_save(mse_intensity_save>eps& mse_intensity_save<mse_intensity_thresh));

%%
figure(3);hist(polyfit_coefs_range_save(polyfit_coefs_range_save<polyfit_coefs_range_thresh_max& polyfit_coefs_range_save>polyfit_coefs_range_thresh_min));
[HParas.R.mean(materialnum),HParas.R.std(materialnum)]=normfit(polyfit_coefs_range_save(polyfit_coefs_range_save<polyfit_coefs_range_thresh_max& polyfit_coefs_range_save>polyfit_coefs_range_thresh_min));

%%
figure(4);hist(polyfit_coefs_intensity_save(polyfit_coefs_intensity_save<polyfit_coefs_intensity_thresh_max&polyfit_coefs_intensity_save>polyfit_coefs_intensity_thresh_min));
[HParas.I.mean(materialnum),HParas.I.std(materialnum)]=normfit(polyfit_coefs_intensity_save(polyfit_coefs_intensity_save<polyfit_coefs_intensity_thresh_max&polyfit_coefs_intensity_save>polyfit_coefs_intensity_thresh_min));
%%

% figure;hist(mse_intensity_save(mse_range_save<1.8))
% [a,b]=normfit(mse_intensity_save(mse_intensity_save>eps& mse_intensity_save<inf))
% figure;hist(mse_intensity_save(mse_intensity_save<inf))
% [a,b]=normfit(mse_intensity_save(mse_intensity_save>eps& mse_intensity_save<inf))
% [a,b]=normfit(mse_intensity_save(mse_intensity_save>eps& mse_intensity_save<150))

% %metal was all clustered around 0
% HParas.I.mean(1)=30;
% HParas.I.std(1)=25; %was actually given as a very small number like 2.9513e-024;
% 
% %was at 40 with a spread of about 20
% HParas.I.mean(2)=40;
% HParas.I.std(2)=20;
% 
% HParas.I.mean(3)=65;
% HParas.I.std(3)=25;

save('Classification_Criteria.mat','HParas');




