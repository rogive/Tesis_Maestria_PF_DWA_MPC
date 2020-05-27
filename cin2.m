function [vel1,vel2,vel3]=cin(Vxr,Vyr,Wr,theta1)
% cinematica del robotino
% la variable l es la distancia que hay entre en centro del robot y el
% centro de la rueda en metros
l=0.29;
% la variable r es el radio de las ruedas en metros
r=0.05;
a1=pi/2;
a2=7*pi/6;
a3=11*pi/6;
% calculo de las velocidades de los motores
% matriz de cinematica por cada rueda j1f

J1f=[1 0 -l;-0.5 (sqrt(3))/2 -l;-0.5 -(sqrt(3))/2 -l];
% J1f=[sin(a1) -cos(a1) -l;sin(a2) -cos(a2) -l;sin(a3) -cos(a3) -l];
% J1f=[sin(theta) -cos(theta) -l;0 -cos(theta+(2/3)*pi) -l;sin(theta-(2/3)*pi) -cos(theta-(2/3)*pi) -l];

% matriz de rotacion
Rot=[cos(theta1) sin(theta1) 0;-sin(theta1) cos(theta1) 0;0 0 1];
% calculo del componente de movimiento J2
J2=[r 0 0;0 r 0;0 0 r];
%ingresar las velocidades en metros por segundo Xr Yr Tetar, las que ya se tienen
% Xr=0.1;
% Yr=0;
% Tetar=0;

% calculo de velocidades en cada motor radianes por segundo , vector phi

phi=J2^-1*J1f*Rot*([Vxr Vyr Wr]');

vel1=phi(1)/8;
vel2=phi(2)/8;
vel3=phi(3)/8;