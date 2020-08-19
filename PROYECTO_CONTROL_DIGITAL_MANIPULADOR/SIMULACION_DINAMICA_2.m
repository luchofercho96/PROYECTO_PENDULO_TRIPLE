clc,clear all,close all;
format long
%% declaracion del tiempo de simulacion
to=0;
ts=0.01;
tf=60;
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
q2(1)=-0;
q1p(1)=0.0;
q2p(1)=0;

T1=-0.*ones(1,length(t));
T2=0*ones(1,length(t));

%% angulos deseados
q1d=pi*sin(0.1*t);
q2d=pi/2*ones(1,length(t));

q1dp=pi*0.1*cos(0.1*t);
q2dp=0*ones(1,length(t));

q1dpp=-pi*0.1*0.1*sin(0.1*t);
q2dpp=0*ones(1,length(t));
KP=diag([1 1]);

KD=diag([5 5]);

for k=1:length(t)
    %% VECTORES GENERALES DE LOS ERRORES
    he=[q1d(k)-q1(k);...
        q2d(k)-q2(k)];
    hep=[q1dp(k)-q1p(k);...
         q2dp(k)-q2p(k)];  
    hdpp=[q1dpp(k);q2dpp(k)];
    
    qp=[q1p(k);q2p(k)];
    %% M ES LA MATRIZ DE MASAS
    M=[l2^2*(m2)*cos(q2(k))^2,0;...
        0,l2^2*(m2)]
    %% C ES LA MATRIZ DE CORIOLIS 
    C=[0,-l2^2*(m2)*sin(2*q2(k))*q1p(k);...
        l2^2*(m2)*sin(2*q2(k))*q1p(k)/2,0];
    %% G ES LA MATRIZ DE GRAVEDAD
    G=[0;g*l2*(m2)*cos(q2(k))];
    %% ES LA MATRIZ DE FRICCIONES
    B=diag([b1 b2]);
    %% LEY DE CONTROL USANDO EL MODELO INVERTIDO
    control=hdpp+KP*tanh(he)+KD*tanh(hep);
     T=M*control+C+G+B*qp;
    %% TORQUES DEL SISTEMA
    torque1(k)=0;
    torque2(k)=0;
    T=[torque1(k);torque2(k)];
    %% DINAMICA DEL SISTEMA
    qpp=inv(M)*(T-C*qp-G-B*qp);
    %% INTEGRACION NUMERICA PARA SACAR VELOCIDADES
    q1p(k+1)=q1p(k)+ts*qpp(1);
    q2p(k+1)=q2p(k)+ts*qpp(2);
    %% INTEGRACION NUMERICA PARA SACAR POSICIONES
    q1(k+1)=q1(k)+ts*q1p(k);
    q2(k+1)=q2(k)+ts*q2p(k);
    
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