%% SIMULACION DE LA PLANTA EN ESPACIOS DE ESTADOS 1
clc,clear all,close all;
%% DECLARACION DEL TIEMPO DE SIMUALCION
to=0;
ts=0.01;
tfinal=10;
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
q2(1)=-1.0;
q1p(1)=0;
q2p(1)=0;

x1(1)=0;
x2(1)=0;
x3(1)=0;
x4(1)=-1.0;


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
  X=[x1(k);x2(k);x3(k);x4(k)];
  
  A=[-(b1)/(l2^2*m2*(cos(q2(k)))^2),(sin(2*q2(k))*q1p(k))/(cos(q2(k))^2);...
      (-sin(2*q2(k))*q1p(k))/2,-b2/(l2^2*m2)]
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
    %% puntos de equilibrio donde se linealizo
    x1e(k)=0;
    x2e(k)=0;
    x3e(k)=0;
    x4e(k)=-pi/2;
    t1e(k)=0;
    t2e(k)=0;
    
%     A_s=[-(b1)/(l2^2*m2*(cos(x4e(k)))^2)+(sin(2*x4e(k))*x2e(k))/(cos(x4e(k))^2),(sin(2*x4e(k))*x1e(k))/(cos(x4e(k))^2),0,(-b1*x1e(k)*2*sin(x4e(k))/(l2^2*m2*(cos(x4e(k)))^3))+((2*cos(2*x4e(k))*cos(x4e(k))+2*sin(x4e(k))*sin(2*x4e(k)))/(cos(x4e(k)))^3)*x1e(k)*x2e(k);...
%         (-2*sin(2*x4e(k))*x1e(k))/2,-b2/(l2^2*m2),0,(-cos(2*x4e(k))*2*x1e(k)^2/2)+g*-cos(x4(k))/l2;...
%         1,0,0,0;...
%         0,1,0,0]
    
    A_s=[-(b1 - l2^2*m2*q2p(k)*sin(2*q2(k)))/(l2^2*m2*cos(q2(k))^2), (2*q1p(k)*sin(q2(k)))/cos(q2(k)), 0, -(2*q1p(k)*(b1*sin(q2(k)) - l2^2*m2*q2p(k)*cos(q2(k))))/(l2^2*m2*cos(q2(k))^3); -q1p(k)*sin(2*q2(k)), -b2/(l2^2*m2), 0, (g*sin(q2(k)))/l2 - q1p(k)^2*cos(2*q2(k)); 1, 0, 0, 0; 0, 1, 0, 0]

    B_s=[1/(l2^2*m2*cos(q2(k))^2), 0; 0, 1/(l2^2*m2); 0, 0; 0, 0];
%     G_s=[0;-g*cos(x4(k))/l2;0;0];
    dX=[x1(k)-x1e(k);x2(k)-x2e(k);x3(k)-x3e(k);x4(k)-x4e(k)];
    dU=[torque1(k)-t1e(k);torque2(k)-t2e(k)];
    XP=A_s*dX+B_s*dU;
    x1(k+1)=x1(k)+ts*XP(1);
    x2(k+1)=x2(k)+ts*XP(2);
    x3(k+1)=x3(k)+ts*XP(3);
    x4(k+1)=x4(k)+ts*XP(4);

     

    
end
figure()
plot(t,q1(1:length(t)),'-r');
hold on
grid on;
plot(t,q2(1:length(t)),'-b');
plot(t,x4(1:length(t)),'--b');
plot(t,x3(1:length(t)),'--r');
legend('q1','q2','q2l','q1l')
figure()
plot(t,x3(1:length(t)),'-r');
hold on;
grid on;
plot(t,x4(1:length(t)),'-b');
plot(t,q1d(1:length(t)),'--r');
plot(t,q2d(1:length(t)),'--b');
legend('q1_l','q2_l','q1d','q2d')
figure()
plot(t,torque1(1:length(t)),'-r');
hold on
grid on;
plot(t,torque2(1:length(t)),'-b');
legend('T1','T2');