function []=Tian_01(ini,obj)
%call:  Tian_01(ini,obj);
%call:  ini=1, random chomosome initialization
%       ini=2, deterministic chromosome initialization
%       obj=1, single objective optimization by total travel distance
%       obj=2, multi-objective, distance modified by turn angle
%generate fractal surface in 2D & place coverage disks
%recursive expansion until all disk outside boundary
%trim disks outside boundary
%GA based trajectory planning
%mutli-objective optimization by penalty
%random or prioritized chromosome initialization

%program management
clc; dbstop if error;

%surface & disk parameters
rng=0.6;%range of surface
cnr=7;%surface corners
fcl=6;%fractal levels
dkr=0.04;%disk radius

%main process

[fig]=InitFigure(rng);%initialize figure with surface range
[frc]=InitFractal(cnr);%initalize fractal with corner points
[frc]=GetFractal(frc,fcl);%generate factal in levels
[frc]=CalFractal(frc);%calculate fractal angle & distance to center
[dsk]=InitDisk(frc,dkr);%initialize disk parameters
[dsk]=GetDisk(frc,dsk,1);%generate disks from 1st to levels
[dsk]=AdjustDisk(frc,dsk);%adjust disks on the boundary
[trj]=GAPath(dsk,ini,obj);%find trajectory using GA
SaveData(dsk,trj,frc);%save disk variables in file

%collection of supporting functions

function [fig]=InitFigure(rng)
%generate the drawing figure
%rng: range
fig=figure('units','normalized',...
  'position',[rand*0.1+0.3 rand*0.1+0.2 0.35 0.35]);
makemenu(fig,'Exit','close gcf;');
%show grid, hold & set axis
axis equal; axis([-1 1 -1 1]*rng); grid on; hold on; 
xlabel('X'); ylabel('Y');%draw label

function [frc]=InitFractal(pts)
%initialize surface parameters from mouse clicks
%pts: number of corner points
frc=[]; frc.scl=0.02; gsc=0.05; i=0;%random frctal scale
while i<pts,
  [x,y,ix]=ginput(1);%moues click on figure
  if ix>0,%button pressed
    i=i+1;%count points
    x=round(x/gsc)*gsc;%location quantization
    y=round(y/gsc)*gsc;
    plot(x,y,'b.');%show points
    frc.xy(:,i)=[x;y];%put into fractal location
  end;
end;

function [frc]=GetFractal(frc,lvl)
%recursively generate surface boundary
%frc: fractal points
%lvl: fractal level
if lvl>0,%continue generate boundary
  xx=size(frc.xy,2);%number of boundary points
  for i=1:xx,%each point
    if i<xx,%ordinary mid-point
      midxy(:,i)=[(frc.xy(1,i)+frc.xy(1,i+1))/2;
                  (frc.xy(2,i)+frc.xy(2,i+1))/2];
    else%last mid-point
      midxy(:,i)=[(frc.xy(1,i)+frc.xy(1,1))/2;
                  (frc.xy(2,i)+frc.xy(2,1))/2];
    end;
    midxy(:,i)=midxy(:,i)+(rand(2,1)-0.5)*frc.scl;%randomize
  end;
  frc.xy(:,1:2:2*xx-1)=frc.xy;%append fractal location
  frc.xy(:,2:2:2*xx)=midxy;%fill intermediate points
  plot(frc.xy(1,:),frc.xy(2,:),'b.');%show
  lvl=lvl-1;%reduce level
  [frc]=GetFractal(frc,lvl);%recursive call
end;
plot(frc.xy(1,:),frc.xy(2,:),'m');%show final boundary
plot([frc.xy(1,1) frc.xy(1,end)],[frc.xy(2,1) frc.xy(2,end)],'r');

function [frc]=CalFractal(frc)
%calculate boundary point distance & angle to center
%frc: list of boundary points
z=size(frc.xy,2);%number of boundary
frc.ctr=mean(frc.xy,2);%center of surface
dx=frc.xy(1,:)-frc.ctr(1);%distance to center
dy=frc.xy(2,:)-frc.ctr(2);
frc.r=sqrt(dx.^2+dy.^2);%distance
frc.q=atan2(dy,dx);%angle to center

function [dsk]=InitDisk(frc,rad)
%initialize disk parameters
%frc: list of boundary points
%rad: disk radius
dsk.rad=rad;%disk radius
dsk.xy(:,1)=frc.ctr;%first disk at center of surface
dsk.inout=0;%boundary flag; 0=in, 1=on, 2=out
dsk=DrawDisk(dsk,1,0);%draw the first disk
plot(dsk.xy(1,1),dsk.xy(2,1),'r+');%show center

function [dsk]=GetDisk(frc,dsk,lvl)
%recursively generate disks, stop if all disks outside surface
%frc: list of boundary points
%dsk: list of disks
%lvl: recusion level
dk=6;%use hexagon pattern
zdsk=size(dsk.xy,2);%original number of disks
ctr=frc.ctr;%center of seed disk is center of surface
r=dsk.rad; qq=pi/dk; qq2=qq*2;%disk radius, angle between disks
d=lvl*2*r*cos(qq);%distance between levels of disks
for i=1:dk,%each disk on the hexagon
  q=i*2*pi/dk;%angle
  xy=[d*cos(q); d*sin(q)]+ctr;%temp disk center
  dsk.xy=[dsk.xy xy];%append disk list
  [dsk]=CheckDisk(frc,dsk,xy);%find disk status
  if lvl>1,%2nd & higher levels
    dskxy=dsk.xy(:,end);%latest disk at start of level
    xy=[d*cos(q+qq2); d*sin(q+qq2)]+ctr;%temp disk center at next hexagon
    dskxyz=(xy-dskxy)/lvl;%intermediate disk center
    for j=1:lvl-1,%each intermediate disk
      xy=dskxy+dskxyz*j;%intermediate center
      dsk.xy=[dsk.xy xy];%append disk list
      [dsk]=CheckDisk(frc,dsk,xy);%find disk status
    end;
  end;
end;
if sum(dsk.inout(end-lvl*dk+1:end)==2)<lvl*dk,%not all outside boundary
  lvl=lvl+1; [dsk]=GetDisk(frc,dsk,lvl);%recursive build disks
end;

function [dsk]=CheckDisk(frc,dsk,xy)
%check if disk is in(0), out(2) of surface and on(3) the boundary
%frc: list of boundary points
%dsk: list of disks
%xy: disk center
z=size(dsk.xy,2);%disk index
dsk.inout(z)=0;%assumed inside surface
dsk=DrawDisk(dsk,size(dsk.xy,2),0);%show disk inside
%find disk characterisitc
dx=xy(1)-frc.ctr(1);%find disk distance to surfac center
dy=xy(2)-frc.ctr(2);
dd=sqrt(dx^2+dy^2); dsk.r(z)=dd;%disk distance to surface center stored
dq=atan2(dy,dx); dsk.q(z)=dq;%disk angle to surface center stored
%find relationship with boundary points
qf=Wrap(abs(frc.q-dq));%angles between disk & boundary points
[qd,id]=sort(qf);%closest boundary to disk by angle
%check if disk is on the boundary
dx=xy(1)-frc.xy(1,:);%find disk distance to boundaries
dy=xy(2)-frc.xy(2,:);
dr=sqrt(dx.^2+dy.^2);%distances to boundaries
[db,ib]=sort(dr); dsk.f(z)=ib(1);%index of nearest boundry point to disks
if (db(1)<dsk.rad) & (dd<frc.r(id(1))),%disk on boundary & inside
%closest point less than disk radius AND 
%disk center to surface center smaller than boundary points
 dsk.inout(z)=1;%on the boundary
 dsk=DrawDisk(dsk,size(dsk.xy,2),1);%show disk on the boundary
end;
%check to see if disk is outside boundary
if (dd>frc.r(ib(1))) | (dd>frc.r(id(1))),%disk away & not overlap
%disk center to surface center larger than nearest boundary
 dsk.inout(z)=2;%outside surface
 dsk=DrawDisk(dsk,size(dsk.xy,2),2);%show disk outside surface
end;

function [dsk]=AdjustDisk(frc,dsk)
%frc: list of boundary points
%dsk: list of disks
z=find(dsk.inout==1);%disk to be adjusted
for i=1:length(z),
    j=z(i);
    dx=dsk.xy(1,j)-frc.xy(1,:);
    dy=dsk.xy(2,j)-frc.xy(2,:);
    dr=sqrt(dx.^2+dy.^2);
    A=find(dr<3*dsk.rad);
    x=frc.xy(1,A); y=frc.xy(2,A);
    [p,S]=polyfit(x,y,1);
    f = polyval(p,x);
    plot(x,f,'r');
    q=atan(p(1));
    Vd=-[cos(dsk.q(j)) sin(dsk.q(j))];
    Md=sqrt(Vd(1)^2+Vd(2)^2);
    Vb1=[dsk.rad*cos(q+pi/2) dsk.rad*sin(q+pi/2)];
    Mb1=sqrt(Vb1(1)^2+Vb1(2)^2);
    Qb1=acos(dot(Vd,Vb1)/(Md*Mb1));
    Vb2=[dsk.rad*cos(q-pi/2) dsk.rad*sin(q-pi/2)];
    Mb2=sqrt(Vb2(1)^2+Vb2(2)^2);
    Qb2=acos(dot(Vd,Vb2)/(Md*Mb2));
    if Qb1<Qb2,
        qb=q+pi/2;
    else
        qb=q-pi/2;
    end;
    xy=dsk.xy(:,z(i));%disk center
    OK=0; scl=0.01;%flag to stop iteration & disk radius scale
    while OK==0,
        xyd=xy+scl*dsk.rad*[cos(qb); sin(qb)];%adjusted disk center
        dx=xyd(1)-frc.xy(1,:);
        dy=xyd(2)-frc.xy(2,:);
        dr=sqrt(dx.^2+dy.^2);%distance of new disk to boundary points
        OK=min(dr)>dsk.rad;%not on boundary
        scl=scl+scl;%otherwise increase adjustment size
    end;
    dsk.xy(:,z(i))=xyd;%update disk list
    [dsk]=DrawDisk(dsk,z(i),3);%show adjusted disk
end;
z=find(dsk.inout==2);%all outside disks
dsk.xy(:,z)=[]; dsk.inout(z)=[];%clean up disk center, inout flag
dsk.r(z)=[]; dsk.q(z)=[]; dsk.f(z)=[];%dist to center, angle & boundary


function [dsk]=DrawDisk(dsk,ix,cr)
%draw disks with colors according to their status
%dsk: list of disks
%ix: the disk to be drawn
%cr: color flag
q=(0:10:360)*pi/180;;%angles
r=dsk.rad; ctr=dsk.xy(:,ix);%disk radius & center
cir=[r*cos(q)+ctr(1); r*sin(q)+ctr(2)];%points on disk
switch cr,%change color for normal or trimmed disks
case 0,%inside boundary
  dsk.fig.cir(ix)=plot(cir(1,:),cir(2,:),...
    'color',[0.4 0.4 0.8],'erasemode','xor');%show disk
  dsk.fig.ctr(ix)=plot(ctr(1),ctr(2),'.',...
  'color',[0.6 0.6 0.8],'erasemode','xor');%show centre
case 1,%on boundary
  set(dsk.fig.cir(ix),'xdata',cir(1,:),'ydata',cir(2,:),...
    'color',[0.8 0.4 0.4]);%show disk
  set(dsk.fig.ctr(ix),'xdata',ctr(1),'ydata',ctr(2),...
    'color',[0.8 0.6 0.6]);%show centre
case 2,%outside boundary
  set(dsk.fig.cir(ix),'xdata',cir(1,:),'ydata',cir(2,:),...
    'color',[0.9 0.9 0.9]);%show disk
  set(dsk.fig.ctr(ix),'xdata',ctr(1),'ydata',ctr(2),...
    'color',[0.9 0.9 0.9]);%show centre
case 3,%adjusted disk
  set(dsk.fig.cir(ix),'xdata',cir(1,:),'ydata',cir(2,:),...
    'color',[0.2 0.8 0.2]);%show disk
  set(dsk.fig.ctr(ix),'xdata',ctr(1),'ydata',ctr(2),...
    'color',[0.6 0.9 0.6]);%show centre  
end;
drawnow;

function [trj]=GAPath(dsk,ini,obj)
%deterine arm path by GA
%define GA parameters
time=cputime;%start time
trj=[];%dummy variable for first call to DrawPath
z=size(dsk.xy,2);%number of disks
switch ini,%generation depends on initialization method
case 1,%random
  GA.G=z*50;
  fname1='Rand chrom init, ';
case 2,%deterministic
  GA.G=z*5;
  fname1='Det chrom init, ';
end;
switch obj,%optimization method
case 1,%single objective
  fname2='Single obj, ';
case 2,%ultiobjective
  fname2='Multi obj, ';
end;
set(gcf,'numbertitle','off','name',[fname1 fname2]);%show approach
GA.P=z; GA.L=z;%population size, number of disks=length of chromosome
GA.pc=0.9; GA.pm=0.9;%crossover, mutation probability
best_fit=realmax;%default best fitness, smaller fitness better
%init chromosomes
for p=1:GA.P,%each chromosome
  Ch(:,p)=GetChrom(ini,dsk,p,GA);%1:random, 2:deterministic
end;
%GA iterations
%evaluate fitness
for g=1:GA.G,%each generation
  for p=1:GA.P,%each chromosome, find fitness
    ix=Ch(:,p);%index to disks
    dx=dsk.xy(1,ix(2:end))-dsk.xy(1,ix(1:end-1));%distance between disks
    dy=dsk.xy(2,ix(2:end))-dsk.xy(2,ix(1:end-1));
    drr=sqrt(dx.^2+dy.^2);%between disk distances
    bdr(p)=sum(drr);%shortest travel distance - raw data
    if obj==2,%multi-objective, modify distance by angle
      dxy=[dx; dy];%prepare to find between disk turn angles
      dq1=dot(dxy(:,1:end-1),dxy(:,2:end));
      dq2=(drr(1:end-1).*drr(2:end));
      dqq=[1 2-dq1./dq2];%cosine angle & normalized, small is better
      drr=drr.*dqq;%distance penalized by turning
    end;
    dr(p)=sum(drr);%total distance for all disks
  end;
  %find elite chromosome & store
  best_dr(g)=min(bdr);%min distance travelled
  [bfit,ix]=min(dr);%smallest fit the best
  if bfit<=best_fit(end),%update best fitness
    best_fit(g)=bfit;%best distance
    best_Ch=Ch(:,ix);%fittest chromosome
    [trj]=DrawPath(trj,dsk,best_Ch,g);%draw trajectory
  else
    best_fit(g)=best_fit(end);%retain previous best fitness
  end;
  %do selection
  dr=dr+max(dr);%penalize max disstance
  fit=dr-min(dr)+rand(1,GA.P)*eps;%adjust to zero
  fit=max(fit)-fit;%smaller fit is better, the invert
  fit=fit/(sum(fit)+eps);%normalize
  se=ceil(fit*GA.P);%number of copies
  [se,ix]=sort(se,'descend');%sort into decending order with index
  if g==1,%show iteration
    bestdr_1=best_dr(1);%min distance
    hdl.t=title(sprintf('Gen:%d/%d Total dis:%4.2f/%4.2f',...
      g,GA.G,best_dr(g),bestdr_1));
  else
    set(hdl.t,'string',sprintf('Gen:%d/%d Total dis:%4.2f/%4.2f',...
      g,GA.G,best_dr(g),bestdr_1));
  end; drawnow;
  %copy into pol of chromosome
  pCh=[];%pool chromosome
  for p=1:GA.P,%each fitness
    pCh=[pCh repmat(Ch(:,ix(p)),1,se(p))];%make multiple copies
  end;
  Ch=pCh(:,1:GA.P);%trim pool chromosome into original pool
  %do crossover
  for p=1:2:GA.P,%each pair
    if rand<GA.pc,%do crossover
      Ch=Crossover(Ch,GA);%do it
    end;
  end;
  %do mutate
  for p=1:GA.P,%each chromosome
    if rand<GA.pm,%do mutation
      Ch(:,p)=Mutate(Ch(:,p),GA);%do it
    end;
  end;
  %do elitism
  re=ceil(rand*GA.P);%pointer
  Ch(:,re)=best_Ch;%restore eilte chromosome
end;
trj.best_Ch=best_Ch;%return best path
etime=cputime-time;%time used for trajectory planning
fname=get(gcf,'name'); fname=sprintf('%s Elapsed time %-6.2f',fname,etime);
set(gcf,'name',fname);%update display of elapsed time

function [ch]=GetChrom(ini,dsk,p,GA)
%generate initial chromosomes
%ini: random or deterministic
%dsk: disk locations
%p:   chromosome index within the population
%GA:  GA parameters
title(sprintf('Chrom%d/%d',p,GA.P));drawnow;%the chromosome generated
switch ini,%which initialization?
case 1,%random init chromosome
  ch=ceil(rand(1,GA.L)*GA.L);%the gene
  [ch]=CheckGeneRep(ch,GA); ch=ch.';%repair repeated genes
case 2,%deterministic
  ch=p;%pre-determined & ordered start
  for i=2:GA.L,%next disks
    z=ch(end);%latest disk selected
    dx=dsk.xy(1,:)-dsk.xy(1,z);%distance to other disks
    dy=dsk.xy(2,:)-dsk.xy(2,z);
    dr=sqrt(dx.^2+dy.^2);%distances
    dq=atan2(dy,dx);%angle to other disks
    cq=cos(dq); cq=2-abs(cq);%normalized absolute cosine angle
    dr=dr.*cq;%distance penalized by angular turn
    [z1,ix]=sort(dr);%sort near disks by distance
    ch(i)=ix(2);%put nearest disk into list
    j=2; while any(ch(i)==ch(1:i-1)),%repair repeated disks
      j=j+1; ch(i)=ix(j);%use next nearby disk
    end;
  end;
end;

function [ch]=CheckGeneRep(ch,GA)
%ch:  the chrmosome to be checked
%GA:  GA parameters
[hs,ix]=hist(ch,1:GA.L);%get histogram
z0=find(hs>1);%any repeatition, histogram>1
while length(z0)>0,%repeatition found
  z1=find(ch==z0(1));%find where repeated
  z2=find(hs==0);%what are the missed genes, histogram=0
  z3=ceil(rand*length(z2));%random pointer to missed genes
  ch(z1(1))=z2(z3);%repair repeated gene by the missed gene
  [hs,ix]=hist(ch,1:GA.L);%get histogram again
  z0=find(hs>1);%any repeatition
end;

function [ch]=Crossover(ch,GA)
%ch:  the chrmosome to be checked
%GA:  GA parameters
r1=ceil(rand*GA.P); r2=ceil(rand*GA.P);%index to chromosome pair
rc=ceil(rand*GA.L);%1 crossover point
c1=ch(:,r1); c2=ch(:,r2);%temp chromosome
if rand<0.5,%crossover start or end section of chromosome
  ch(1:rc,r1)=c2(1:rc);%exchange start section disk indices at rc point
  ch(1:rc,r2)=c1(1:rc);
else
  ch(rc:end,r1)=c2(rc:end);%exchange end section disk indices at rc point
  ch(rc:end,r2)=c1(rc:end);
end;
ch(:,r1)=CheckGeneRep(ch(:,r1),GA);%repair repeated genes
ch(:,r2)=CheckGeneRep(ch(:,r2),GA);

function [ch]=Mutate(ch,GA)
%ch:  the chrmosome to be checked
%GA:  GA parameters
m1=ceil(rand*GA.L); m2=ceil(rand*GA.L);%pointer
m=ch(m1); ch(m1)=ch(m2); ch(m2)=m;%exchange 2 genes

function [trj]=DrawPath(trj,dsk,bCh,ix)
%trj: variable to store the drawing handle
%dsk: disks
%bCh: best chromosome
%ix:  graph handle index
xy=dsk.xy(:,bCh);%disk centers
if ix==1,%first drawing at first generation
  trj.plt=plot(xy(1,:),xy(2,:),'r','erasemode','none',...
    'linewidth',2);
  trj.s=plot(xy(1,1),xy(2,1),'go','erasemode','none',...
    'linewidth',4);
  trj.e=plot(xy(1,end),xy(2,end),'ro','erasemode','none',...
    'linewidth',4);
else%subsequent drawing at later generations
  set(trj.plt,'xdata',xy(1,:),'ydata',xy(2,:));
  set(trj.s,'xdata',xy(1,1),'ydata',xy(2,1));
  set(trj.e,'xdata',xy(1,end),'ydata',xy(2,end));
end;

function [q]=Wrap(q)
%confine angles between pi & -pi
%q: angle to be wrapped
while q>+pi, q=q-2*pi; end;
while q<-pi; q=q+2*pi; end;

function []=SaveData(dsk,trj,frc)
[FileName,PathName,ix]=uiputfile('*.mat','Save as');
if ix>0,
    eval(sprintf('save ''%s%s'' dsk trj frc',PathName,FileName));
end;

