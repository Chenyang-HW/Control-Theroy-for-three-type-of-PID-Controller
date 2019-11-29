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
% T=1;
% [Numd,Dend]=c2dm(G,1,T,'zoh');
% Gd=tf(Numd,Dend,1);
% %step(Gd);
% % pzmap(Gd);
% % zgrid
% Gdf=1/(1+Gd);
% stepinfo(Gdf)
% step(Gdf)

