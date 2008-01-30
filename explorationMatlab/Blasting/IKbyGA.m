function [AngCh]=IKbyGA(robot,dsk,trj)
% close all;clear all;clc;
% addpath('C:\MATLAB\R2006b\toolbox\TOOLBOX_robot');
%%
robot=densoArmD(6); % Load the parameters of densoArm with 6-DFO (robot defination)
[dskR,frcR,trj]=CoordinatTrans();%load trjectory
dsk.xyz=dskR.xyz;
%%
%%%%%%%%%%%% Defineation of Main Parameters %%%%%%%%%%%%%%%%%
GA.L=robot.n;% number of joint angles=number of robot DOF,length of chromosome
GA.pc=0.9;%crossoverprobability
GA.pm=0.9;% mutation probability
best_fit=realmax;%default best fitness, smaller fitness better
GA.G=100;%max Gen
GA.NC=50;%number of chromosome
Base=[500; 375; 370];%robot base
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%%%%%%%%%%%main process%%%%%%%%%%%%

[AngCh]=GAIK(robot,dsk,trj);
[AngCh]=GetChrom(GA);
[AngCh]=Crossover(AngCh,GA);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
function [AngCh]=GAIK(robot,dsk,trj)
Base=[500; 375; 370];%robot base
%deterine joint angles by GA
time=cputime;%start time
GA.L=robot.n;% number of joint angles=number of robot DOF,length of chromosome
GA.pc=0.7;%crossoverprobability
GA.pm=0.7;% mutation probability
GA.G=50;%max Gen
GA.NC=1000;%number of chromosome
best_fit=realmax;%default best fitness, smaller fitness better
Init_AngCh=[0 -75 160 0 30 0 ]'; %initinal pose(min pose);
%init chromosomes
AngCh=GetChrom(GA);

%evaluate fitness
Best_AngCh=[];
for ix=1:length(trj.best_Ch)
    AngCh=GetChrom(GA);%init chromosomes
    best_fit=realmax;%default best fitness, smaller fitness better
    targetnormal=[-1;0;0];
    for g=1:GA.G,%each generation
        OF=[];%store distance
        dr=[];
        for p=1:GA.NC,%each chromosome, find fitness
            T=fkine(robot, deg2rad(AngCh(:,p))); %Fk:postion
            Targetpoint=dsk.xyz(1:3,trj.best_Ch(ix))+[300;0;0];%desired points with streamlength
            Target=T(1:3,end)*1000;%target in mm
            dR=Targetpoint-(Target+Base);%coordinate difference
            drr=sqrt(sum(dR.^2));%distance to target
            Normal=T(1:3,3)*1000;
            dN=Normal-targetnormal*1000;
            dnn=sqrt(sum(dN.^2));
            OFM=0.5*drr+0.23*dnn;
            OF=[OF OFM];%append distance for each chromosome 
            dr=[dr drr];
        end;
       
        %find elite chromosome & store
        [bfit,ixx]=min(OF);%smallest fit the best
        if bfit<=best_fit(end),%update best fitness
            best_fit(g)=bfit;%best distance
            best_AngCh=AngCh(:,ixx);%fittest chromosome
        else
            best_fit(g)=best_fit(end);%retain previous best fitness
        end;
        clc; disp([ix*1000 g*1000 best_fit(end)]);
        %do selection
        fit=OF-min(OF)+rand(1,GA.NC)*eps;%adjust to zero
        fit=max(fit)-fit;%smaller fit is better, the invert
        Probfit=fit/(sum(fit)+eps);%normalize
        se=ceil(Probfit*GA.NC);%number of copies
        [se,ise]=sort(se,'descend');%sort into decending order with index

        %copy into pol of chromosome
        pAngCh=[];%pool chromosome 
        for p=1:GA.NC,%each fitness
            pAngCh=[pAngCh repmat(AngCh(:,ise(p)),1,se(p))];%make multiple copies
        end;
        AngCh=pAngCh(:,1:GA.NC);%trim pool chromosome into original pool
        %   do crossover
        for p=1:2:GA.NC,%each pair
            if rand<GA.pc,%do crossover
                AngCh=Crossover(AngCh,GA);%do it
            end;
        end;
        %do mutate
        for p=1:GA.NC,%each chromosome
            if rand<GA.pm,%do mutation
                for i=1:6;
                    AngCh(i,p)=AngCh(i,p)+rand;
                end
%                 AngCh(:,p)=Mutate(AngCh(:,p),GA);%do it
            end;
        end;
        %do elitism
        re=ceil(rand*GA.NC);%pointer
        AngCh(:,re)=best_AngCh;%restore eilte chromosome
    end;
    clc; disp(ix); disp(best_AngCh);
    Best_AngCh=[Best_AngCh best_AngCh];%return best jont angles
end;

Best_AngCh=[Best_AngCh best_AngCh];%return best jont angles
SaveDataAngbyGA(Best_AngCh);
%%
function [AngCh]=GetChrom(GA)
%generate initial chromosomes under the robot condiation
%GA.NC: number of chrmosome
%GA: GA parameters
GA.NC=1000;
AngCh(1,:)=-170+rand(1,GA.NC)*340;
AngCh(2,:)=-90+rand(1,GA.NC)*225;
AngCh(3,:)=-80+rand(1,GA.NC)*245;
AngCh(4,:)=-185+rand(1,GA.NC)*370;
AngCh(5,:)=-120+rand(1,GA.NC)*240;
AngCh(6,:)=-360+rand(1,GA.NC)*720;

%%
function [AngCh]=Crossover(AngCh,GA)
%AngCh:  the chrmosome to be checked
%GA:  GA parameters

% r1=ceil(rand*GA.NC); r2=ceil(rand*GA.NC);%index to chromosome pair
% for i=1:6
%     r=rand;
%     AngCh(i,r1)=AngCh(i,r1)*r+AngCh(i,r2)*(1-r);
%     AngCh(i,r2)=AngCh(i,r1)*(1-r)+AngCh(i,r2)*r;
% end