clc;
clear;

J=0.013;
yita=0.3076;
v=0.11;
I=0.0195;
m=0.284;
g=9.81;
tL=0.02;

L=1;
k=1/pi;
R=0.2;
miu=10/(6*pi*pi);

%fan system
%input is e(t); 
%output is T(t);%state space
A1=[(-R/I) (-k/I);(k/J) (-miu/J)];
B1=[(1/I);0];
C1=[k 0];
D1=[0];
poles=eig(A1);

%Lapulase Thransform sys1
[N1,D1]=ss2tf(A1,B1,C1,D1);
G1=tf(N1, D1);
% step(G1)

%air system sys2
%input is F(t)
%output is F(t-tL)
G_2=tf(N1,D1,'inputDelay',0.02);
G2=series(G_2,v);%T(t-tL) to F(t-tL)

%palte system sys3
%input is F(t-L)
%output is sita(t)
A3=[0 1;(-m*g)/I (-yita)/I];
B3=[0;1];
C3=[1 0];
D3=[0];
[N3,D3]=ss2tf(A3,B3,C3,D3);
G3=tf(N3, D3);

G=G2*G3;

%Transfer system to discrete system
T=0.05;
[Numd,Dend]=c2dm(G,1,T,'zoh');
Gd=tf(Numd,Dend,1/20);
%step(Gd);
% pzmap(Gd);
% zgrid
Gc1=tf([1 -0.76], [1 -1], 0.05);%PI transfer function
Gc2=series(Gc1,19.01);
Gdpi=series(Gc2,Gd);%output for open loop

% %Root locus in PI
% rlocus(Gdpi)
% zgrid

%PID controller
kp=14.44;ki=4.57;kd=1;
r1=0.6; r2=0.8;
Gcpid1=tf([1 -(r1+r2) r1*r2],[1 -1 0], 0.05);
Gcpid2=series(Gcpid1,kp+ki+kd);
Gdpid=series(Gcpid2,Gd);



aefa_lag=4.2; tao_lag=26;
aefa_lead=1.7; tao_lead=3;
%lag transfer function
Nlag=[tao_lag 1];
Dlag=[aefa_lag*tao_lag 1];
Glag=tf(Nlag,Dlag);
[Numdlag,Dendlag]=c2dm(Glag,1,T,'zoh');
Gdlag=tf(Numdlag,Dendlag,1/20);

%lead transfer function
Nlead=[tao_lead 1];
Dlead=[aefa_lead*tao_lead 1];
Glead=tf(Nlead,Dlead);
[Numdlead,Dendlead]=c2dm(Glead,1,T,'zoh');
Gdlead=tf(Numdlead,Dendlead,1/20);

Gdpidls=Gdpid*Gdlag*Gdlead;
bode(Gdpidls)
Gf=series(Gdpidls,Gd);

% bode(Gd)
% Gdf=1/(1+Gf);
% stepinfo(Gdf)
% step(Gdf)
% 

