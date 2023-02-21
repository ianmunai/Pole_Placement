clear
clc
close all

num=10;
den=conv(conv([1 1],[1 2]),[1 3]);

sys=tf(num,den);

%Obtain the state space model, eigenvalues and eigenvector
[A,B,C,D]=tf2ss(num,den);
[V,X]=eig(A)

%design the state space controller, u=-Kx, giving suitable pole locations;

%this is done using pole placement
%desired closed loop eigenvalues
P=[-4 -6 -7];
P2=[-3 -4+5i -4-5i];

%solve for K using place command
K=place(A,B,P2);
%can also use acker command, it is however not numerically reliable
% K2=acker(A,B,P)

%check for closed loop eigenvalues
Acl=A-B*K;
Ecl=eig(Acl)

%create closed loop system and compare step responses for both systems
sysc=ss(Acl,B,C,D);
figure
step(sys)
grid on

figure
step (sysc)
grid on

%solve for Kr
Kdc=dcgain(sysc);
Kr=1/Kdc;

%build scaled input closed system
sysc_scaled=ss(Acl,B*Kr,C,D);
figure
step(sysc_scaled)
grid on
