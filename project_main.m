% ME751 AC Simulation
% Main loop
disp('ME751 Adaptive Control Simulation');
while input('Continue (Y/N)? : ', 's')=='y',
clear
close all
% Input constraint
maximumU=10;
% Plant
B=[0.5];
A=[1 -1.8 0.9];
disp('Open loop poles are: ');
roots(A)
h1=-input('Enter the desired location of the closed-loop pole: ');
alpha=input('Enter alpha (P(0)=alpha*I): ');
lambda=input('Enter lambda: ');
steps=input('Enter the number of steps (0-5): ');
noiseRatio=input('Enter the ratio of the noise std. deviation to the step size: ');
% Initialize variables
NK=200; % Time Duration in samples
oldP=alpha*eye(3);
oldTheta=[.5*A(2); 1.5*A(3); .5*B(1)];
a1=oldTheta(1)*ones(1,2);
a2=oldTheta(2)*ones(1,2);
b1=oldTheta(3)*ones(1,2);
L1=zeros(1,2);
P1=alpha*ones(1,2);
if steps==0
    r=zeros(NK,1);
else
    r=zeros(10,1);
    for i=1:steps,
        r=[r' ones(0.5*(NK-10)/steps,1)' -ones(0.5*(NK-10)/steps,1)']';
    end
end
y=zeros(NK,1);
y(1:2)=[-0.1 -0.1]; % Initial offset of output
u=zeros(NK,1);
e=zeros(NK,1);
randn('seed', 8);
for i=3:NK,
    y(i)=B(1)*u(i-1)-A(2)*y(i-1)-A(3)*y(i-2)+noiseRatio*randn;
    phi=[-y(i-1); -y(i-2); u(i-1)];
    [newTheta, newP, newL, e(i)]=rls751(oldTheta, oldP, phi, lambda, y(i));
    oldP=newP;
    oldTheta=newTheta;
    P1(i)=newP(1,1);
    L1(i)=newL(1);
    a1(i)=newTheta(1);
    a2(i)=newTheta(2);
    b1(i)=newTheta(3);
    u(i)=u(i-1)+((1+h1)/b1(i))*((r(i)-y(i))+a1(i)*(r(i-1)-y(i-1))+a2(i)*(r(i-2)-y(i-2)));
    if abs(u(i))>maximumU
        u(i)=sign(u(i))*maximumU;
    end
    %disp(i);
end
% Plot y, u and r
subplot(2,2,1)
t=[1:NK]';
ymin=min([min(y) min(u) min(r)]);
ymax=max([max(y) max(u) max(r)]);
%axis([0 120 ymin ymax]);
plot(t, y, 'k');
title('Setpoint, Output and Input');
hold on
stairs(t, u, 'g')
stairs(t, r, 'r');
legend('y', 'u', 'r');
hold off
subplot(2,2,2)
% Plot e and parameter estimates
ymin=min([min(e) min(a1) min(a2) min(b1)]);
ymax=max([max(e) max(a1) max(a2) max(b1)]);
axis([1 NK ymin ymax]);
plot(t, e, 'g', t, a1, 'r');
hold on
plot(t, a2, 'k', t, b1, 'b');
title('Prediction error and parameter estimates');
legend('e', 'a_1', 'a_2', 'b_1');
hold off
subplot(2,2,3)
plot(t, P1);
title('First Element of Covariance Matrix P');
xlabel('Sample number')
subplot(2,2,4)
plot(t, L1);
title('First Element of Gain Vector L');
xlabel('Sample number')
end
