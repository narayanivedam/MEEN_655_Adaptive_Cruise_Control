function distrack
close all
t0=0;
tf=200;
f0=0.1;
f1=5;
f2=0.25;
M=1650;
ga=3.4;
ka=10;
rw=1;
g=9.8;
FTR=ga*ka/(M*rw);
f0=f0/M;
f1=f1/M;
f2=f2/M;
k1=1;
k2=1;
k=2;
alpha=1.1;%sliding model para
ks=1;
xd=10;
vx0=[20;30];%initialized at v0=56mph, assume the sensing radius is 10 m
[t,vx]=ode45('fxx',[t0 tf],vx0);
vf=zeros(size(t),1);
vfd=zeros(size(t),1);
for i=1:size(t)
if t(i)<=50
    vf(i)=17.8;
    vfd(i)=0;
elseif (t(i)>50) & (t(i)<=80)
    vf(i)=17.8+0.1*(t(i)-50);
    vfd(i)=1;
elseif (t(i)>80) & (t(i)<=100)
    vf(i)=20.8;
    vfd(i)=0;
elseif (t(i)>100) &(t(i)<=120) 
    vf(i)=20.8-0.1*(t(i)-100);
    vfd(i)=-1;
else
    vf(i)=18.8;
    vfd(i)=0;
end   %speed of the front car
end
e=zeros(size(t),2);
e(:,1)=vx(:,1)-xd*ones(size(t),1);
e(:,2)=vf-vx(:,2);
u=zeros(size(t),1);
%u=(1/FTR)*((f0+f1*vx(:,2)+f2*vx(:,2).^2)+vfd+k1*e(:,1)+k2*e(:,2));%IOL
for i=1:size(t)
    lfv=e(i,2)*(vfd(i)+f0+f1*vx(i,2)+f2*vx(i,2)^2);
  % lfv=e(i,1)*e(i,2);
    lfv+0.5*k*(e(i,2)^2);
 if (lfv+0.5*k*(e(i,2)^2)<0)
      u(i)=0;
      %u(i)=(1/(FTR*e(i,2)))*(lfv+0.5*k*(e(i,2)^2));%CLF
 else
      if e(i,2)~=0
      u(i)=(1/(FTR*e(i,2)))*(lfv+0.5*k*(e(i,2)^2));%CLF
      else
      % u(i)=(1/(FTR))*((vfd(i)+f0+f1*vx(i,2)+f2*vx(i,2)^2)+10);
      u(i)=0;
      end
 end
end%CLF
%u=(1/FTR)*((vfd+f1*e(:,2)+k1*e(:,1)+k2*e(:,2)));%linear controller
%u=(1/FTR)*(1*e(:,2)+vfd+(f0+f1*vx(:,2)+f2*vx(:,2).^2)+1.1*sign((e(:,2)+1*e(:,1))));%sliding mode controller
v=zeros(size(t),1);
v=vfd-FTR*u+(f0+f1*vx(:,2)+f2*vx(:,2).^2);
figure(1)
plot(t,v,'b');
hold on
%axis([-10 tf -10 200]);
figure(2)
input=u*FTR/(M*g);
plot(t,input,'r');
hold on
%axis([-10 tf -10 1200]);
figure(3)
plot(t,vx(:,1),'b');
%axis([45 55 0 20]);
hold on
figure(4)
plot(t,vf,'r');
hold on
plot(t,vx(:,2),'k');
hold on
%axis([70 110 40 80]);
save NLDisclf.mat t vx vf input v
end
