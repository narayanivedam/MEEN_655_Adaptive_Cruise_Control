function xdot=fxx(t,vx)
f0=0.1;
f1=5;
f2=0.25;
M=1650;
ga=3.4;
ka=10;
rw=1;
FTR=ga*ka/(M*rw);
f0=f0/M;
f1=f1/M;
f2=f2/M;
alpha=1.1;
ks=1;
% vf=40+2*sin(0.1*t);
% vfd=0.2*cos(0.5*t);
if t<=50
    vf=17.8;
    vfd=0;
elseif (t>50) & (t<=80)
    vf=17.8+0.1*(t-50);
    vfd=1;
elseif (t>80) & (t<=100)
    vf=20.8;
    vfd=0;
elseif (t>100) &(t<=120) 
    vf=20.8-0.1*(t-100);
    vfd=-1;
else
    vf=18.8;
    vfd=0;
end   %speed of the front car
xd=10;%disired distance
e(1)=vx(1)-xd;
e(2)=vf-vx(2);   %transform to normal form
k1=1;
k2=1;
k=2;
%lfv=e(1)*e(2)+e(2)*(vfd+f0+f1*vx(2)+f2*vx(2)^2);
%lfv=e(1)*e(2);
lfv=e(2)*(vfd+f0+f1*vx(2)+f2*vx(2)^2);
%u=(1/FTR)*(1*e(2)+vfd+(f0+f1*vx(2)+f2*vx(2)^2)+1.1*sign((e(2)+1*e(1))));%sliding mode controller
% u=(1/FTR)*((vfd+f1*e(2)+k1*e(1)+k2*e(2)));%linear controller
 if (lfv+0.5*k*(e(2)^2)<0)
      u=0;
     % u=(1/(FTR*e(2)))*(lfv+0.5*k*(e(2)^2));%CLF
 else
      if e(2)~=0
         u=(1/(FTR*e(2)))*(lfv+0.5*k*(e(2)^2));%CLF
      else 
        % u=(1/(FTR))*((vfd+f0+f1*vx(2)+f2*vx(2)^2)+10);
        u=0;
      end
 end
%  lfv-FTR*e(2)*u;
%u=(1/FTR)*((f0+f1*vx(2)+f2*vx(2)^2)+vfd+k1*e(1)+k2*e(2));%io control law
edot(1)=e(2);
edot(2)=vfd-FTR*u+(f0+f1*vx(2)+f2*vx(2)^2);
%edot=edot';
xdot(1)=edot(1);
xdot(2)=vfd-edot(2);
xdot=xdot';
end