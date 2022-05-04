function vtrack
close all
f0=0.1;
f1=5;
f2=0.25;
M=1650;
ga=3.4;
ka=10;
rw=1;
FTR=ga*ka/(M*rw);
g=9.8;
k1=1;
vr=17.8;%40miles/hour
t0=0;
tf=400;
v0=20;%initialized at v0=35mph, assume the sensing radius is 10 m
%roots([f2/M f1/M f0/M])
[t,v]=ode45('fxx',[t0,tf],v0);
u=zeros(size(t),1);
for i=1:size(t)
if -(v(i)-vr)*(f0+f1*v(i)+f2*v(i)^2)/M+k1*(v(i)-vr)^2<=0
    u(i)=0;
else
    u(i)=(1/FTR)*((f0+f1*v(i)+f2*v(i)^2)/M-k1*(v(i)-vr));
end                      %CLF
end
%u=(1/FTR)*((f0+f1*v+f2*v.^2)/M-k1*(v-vr)); %IO
%u=(f1*vr*ones(size(t),1)-(v-vr*ones(size(t),1)))/FTR;%linear
ac=FTR*u-(f0+f1*v+f2*v.^2)/M;
figure(1)
input=u*FTR/(M*g);
plot(t,input,'b');
figure(2)
plot(t,ac,'r');
figure(3)
plot(t,v,'r');
%axis([45 55 0 20]);
hold on
line([0 tf],[40 40],'color','b');
hold off
save NLvclf.mat t v input ac
end