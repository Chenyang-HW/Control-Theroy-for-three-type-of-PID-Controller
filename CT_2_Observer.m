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
G1=tf(N1,D1);
% step(G1)
%discrete-time linear state feedback controller
syms Nbar1;
eig(A1);
p11=-17+3i;
p12=-17-3i;
K1 = place(A1,B1,[p11 p12]);
Nbar1 =-1/(C1*inv(A1-(B1*K1))*B1);
[Nls1,Dls1]=ss2tf((A1-(B1*K1)),Nbar1*B1,C1,0);
Gls1=tf(Nls1, Dls1);
%observer
L1=place(A1',C1',[-30 -24])';%observer gain
Acl1=[A1-(B1*K1) B1*K1; zeros(size(A1)) A1-(L1*C1)];
Bcl1=Nbar1*[B1;zeros(length(A1),1)];
Ccl1=[C1 zeros(1,length(A1))];
[Ncl1,Dcl1]=ss2tf(Acl1,Bcl1,Ccl1,0);
Gcl1=tf(Ncl1,Dcl1);

%air system sys2
%input is F(t)
%output is F(t-tL)
%G_2=tf(N1,D1,'inputDelay',0.02);
G_2=tf(Ncl1,Dcl1,'inputDelay',0.02);
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
% discrete-time linear state feedback controller
syms  Nbar3;
eig(A3);
p31=-145+3i;
p32=-145-3i;
K3 = place(A3,B3,[p31 p32]);
Nbar3 =-1/(C3*inv(A3-(B3*K3))*B3);
[Nls3,Dls3]=ss2tf(A3-(B3*K3),Nbar3*B3,C3,0);
Gls3=tf(Nls3, Dls3);
%observer
L3=place(A3',C3',[-150 -148])';%observer gain
Acl3=[A3-(B3*K3) B3*K3; zeros(size(A3)) A3-(L3*C3)];
Bcl3=Nbar3*[B3;zeros(length(A3),1)];
Ccl3=[C3 zeros(1,length(A3))];
[Ncl3,Dcl3]=ss2tf(Acl3,Bcl3,Ccl3,0);
Gcl3=tf(Ncl3,Dcl3);

%G=G2*G3;
G=G2*Gcl3;
%Transfer system to discrete system
T=0.05;
[Numd,Dend]=c2dm(G,1,T,'zoh');
Gd=tf(Numd,Dend,1);
Gdf=1/(1+Gd);
stepinfo(Gdf)
step(Gdf)
