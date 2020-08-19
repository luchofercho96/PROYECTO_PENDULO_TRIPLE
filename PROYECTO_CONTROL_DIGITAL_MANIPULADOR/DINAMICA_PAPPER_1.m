clc,clear all,close all;
%% declaracion del tiempo de simulacion
to=0;
ts=0.01;
tf=2;
t=to:ts:tf;
%% Variables del sistema longitudes y masas
m1=1;
m2=0.5;
l1=0.1;
l2=0.5;
g=9.81;
b1=0.2;
b2=0.2;
%% declaracion de los estados iniciales del robot
q1(1)=0;
q2(1)=-pi/2;
q1p(1)=0.0;
q2p(1)=0;

T1=-0.*ones(1,length(t));
T2=0*ones(1,length(t));

%% angulos deseados
q1d=pi/2*ones(1,length(t));
q2d=pi/2*ones(1,length(t));

KP=diag([1 1]);

KD=diag([2 2]);
I=eye(2);
Z=0*ones(2,2);
for k=1:length(t)
    %% VECTORES GENERALES DE LOS ERRORES
%     he=[q1d(k)-q1(k);...
%         q2d(k)-q2(k)];
%     hep=[q1dp(k)-q1p(k);...
%          q2dp(k)-q2p(k)];  
%     hdpp=[q1dpp(k);q2dpp(k)];
    
   q=[q1p(k);q2p(k);q1(k);q2(k)];

    %% M ES LA MATRIZ DE MASAS
    M=[l2^2*(m2)*cos(q2(k))^2,0;...
        0,l2^2*(m2)];
    %% C ES LA MATRIZ DE CORIOLIS 
    C=[0,-l2^2*(m2)*sin(2*q2(k))*q1p(k);...
        l2^2*(m2)*sin(2*q2(k))*q1p(k)/2,0];
    %% G ES LA MATRIZ DE GRAVEDAD
    G=[0;g*l2*(m2)*cos(q2(k))];
    %% ES LA MATRIZ DE FRICCIONES
    B=diag([b1 b2]);
    %% TORQUES DEL SISTEMA
    torque1(k)=0;
    torque2(k)=0;
    T=[q1d(k);q2d(k)];
    
    %% DINAMICA DEL SISTEMA
    qp=[-inv(M)*(C+KD+B),-inv(M)*KP;I,Z]*q+[inv(M)*KP;Z]*T+[-inv(M)*G;0;0];
    
    %% INTEGRACION NUMERICA PARA SACAR VELOCIDADES
    q1p(k+1)=q1p(k)+ts*qp(1);
    q2p(k+1)=q2p(k)+ts*qp(2);
    %% INTEGRACION NUMERICA PARA SACAR POSICIONES
    q1(k+1)=q1(k)+ts*qp(3);
    q2(k+1)=q2(k)+ts*qp(4);
    
end
figure()
plot(t,q1(1:length(t)),'-r');
hold on
grid on;
plot(t,q2(1:length(t)),'-b');
plot(t,q1d(1:length(t)),'--r');
plot(t,q2d(1:length(t)),'--b');
legend('q1','q2','q1d','q2d')
figure()
plot(t,torque1(1:length(t)),'-r');
hold on
grid on;
plot(t,torque2(1:length(t)),'-b');
legend('T1','T2');