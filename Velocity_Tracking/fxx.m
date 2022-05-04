function xdot=fxx(t,vx)
f0=0.1;
f1=5;
f2=0.25;
M=1650;
ga=3.4;
ka=10;
rw=1;
FTR=ga*ka/(M*rw);
k1=1;
vr=17.8;%40miles/hour
%u=(f1*vr-(vx-vr))/FTR;%(1/FTR)*((f1)*vx-k1*vx);%linear
if -(vx-vr)*(f0+f1*vx+f2*vx^2)/M+k1*(vx-vr)^2<=0
    u=0;
else
    u=(1/FTR)*((f0+f1*vx+f2*vx^2)/M-k1*(vx-vr));
end
%u=(1/FTR)*((f0+f1*vx+f2*vx^2)/M-k1*(vx-vr));%io linearization control law
xdot=FTR*u-(f0+f1*vx+f2*vx^2)/M;
xdot=xdot';
end