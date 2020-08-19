%% SIMULACION DE LA PLANTA EN ESPACIOS DE ESTADOS 1
clc,clear all,close all;
%% DECLARACION DEL TIEMPO DE SIMUALCION
to=0;
ts=0.01;
tfinal=2;
t=to:ts:tfinal;
%% VARIABLES DEL SISTEMA LONGITUDES Y MASAS
m1=1;
m2=0.5;
l1=0.1;
l2=0.5;
g=9.81;
b1=0.2;
b2=0.2;
%% DECLARACION DE LOS ESTADOS INICIALES DEL ROBOT
q1(1)=0;
q2(1)=-1.55;
q1p(1)=0;
q2p(1)=0;

q1_l(1)=0;
q2_l(1)=-1.55;
q1p_l(1)=0;
q2p_l(1)=0;

%% VECTORES DE ENTRADAS DEL SISTEMA
T1=0*ones(1,length(t));
T2=0*ones(1,length(t));
%% SENALES DE REFRENCIA DEL CONTROLADOR
q1d=pi*sin(0.1*t);
q2d=pi/2*ones(1,length(t));

q1dp=pi*0.1*cos(0.1*t);
q2dp=0*ones(1,length(t));

q1dpp=-pi*0.1*0.1*sin(0.1*t);
q2dpp=0*ones(1,length(t));
for k=1:length(t)
  torque1(k)=0;
  torque2(k)=0;
  T=[torque1(k);torque2(k)];
  qp=[q1p(k);q2p(k)];
  qp_l=[q1p_l(k);q2p_l(k)];
  
  A=[-(b1)/(l2^2*m2*(cos(q2(k)))^2),(sin(2*q2(k))*q1p(k))/(cos(q2(k))^2);...
      (-sin(2*q2(k))*q1p(k))/2,-b2/(l2^2*m2)];
  B=[1/(l2^2*m2*(cos(q2(k)))^2),0;...
      0,1/(l2^2*m2)];
  G=[0;-g*cos(q2(k))/l2];
  
  qpp=A*qp+B*T+G;
  
  %% INTEGRACION NUMERICA PARA SACAR VELOCIDADES
    q1p(k+1)=q1p(k)+ts*qpp(1);
    q2p(k+1)=q2p(k)+ts*qpp(2);
    %% INTEGRACION NUMERICA PARA SACAR POSICIONES
    q1(k+1)=q1(k)+ts*q1p(k);
    q2(k+1)=q2(k)+ts*q2p(k);
    
    A_l=[(-b1/(l2^2*m2)),(2*q1p_l(k)*q2_l(k));...
         -q1p_l(k)*q2_l(k),(-b2)/(l2^2*m2)];
     B_l=[1/(l2^2*m2),0;...
         0,1/(l2^2*m2)];
     G_l=[0;-g/l2];
     
       
  qpp_l=A_l*qp_l+B_l*T+G_l;
       %% INTEGRACION NUMERICA PARA SACAR VELOCIDADES
    q1p_l(k+1)=q1p_l(k)+ts*qpp_l(1);
    q2p_l(k+1)=q2p_l(k)+ts*qpp_l(2);
    %% INTEGRACION NUMERICA PARA SACAR POSICIONES
    q1_l(k+1)=q1_l(k)+ts*q1p_l(k);
    q2_l(k+1)=q2_l(k)+ts*q2p_l(k);
    
end
figure()
plot(t,q1(1:length(t)),'-r');
hold on
grid on;
plot(t,q2(1:length(t)),'-b');
% plot(t,q1d(1:length(t)),'--r');
% plot(t,q2d(1:length(t)),'--b');
% legend('q1','q2','q1d','q2d')
plot(t,q1_l(1:length(t)),'--r');
plot(t,q2_l(1:length(t)),'--b');
legend('q1','q2','q1l','q2l')
figure()
plot(t,torque1(1:length(t)),'-r');
hold on
grid on;
plot(t,torque2(1:length(t)),'-b');
legend('T1','T2');