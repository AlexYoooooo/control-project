clc;
clear;


rg1=1; %gear ratio
rg2=1;
tauc1=10; %friction
tauc2=0;
tauL1=1000; %torque limit
tauL2=1000;

%mass&length
m1=10;
m2=5;
l1=0.5;
l2=0.5;

%inertia
j1=m1*l1^2/rg1^2;    
j2=m2*l2^2/rg2^2;

% Simulation parameters
T=0.001; % sampling period
N=1000; % number of points
t=T*(0:(N-1)); % time vector

%data variables
theta1=zeros(1,N);  % position 1
theta1Dot=zeros(1,N); % speed 1
theta1DDot=zeros(1,N); % acceleration 1
theta2=zeros(1,N);  % position 2
theta2Dot=zeros(1,N); % speed 2
theta2DDot=zeros(1,N); % acceleration 2

theta1Des=zeros(1,N); % desired position 1
theta1DotDes=zeros(1,N); % desired speed 1
theta1DDotDes=zeros(1,N); % desired acceleration 1
theta2Des=zeros(1,N); % desired position 2
theta2DotDes=zeros(1,N); % desired speed 2
theta2DDotDes=zeros(1,N); % desired acceleration 2

u1=zeros(1,N);  % total control signal 1
u2=zeros(1,N);  % total control signal 2
taum1=zeros(1,N);  % torque output 1
taum2=zeros(1,N);  % torque output 2


%future controller here, currently set constant tau for openloop

for i=1:N
	u1(i)=0;  % torque output 1
	u2(i)=1;  % torque output 2
end







%construct robot dynamics

for i=1:N-1


	%matric A
	Aa=j1*rg1^2+(m1+m2)*l1^2+m2*l2^2+2*m2*l1*l2*cos(theta2(i));
	Ab=m2*l2^2+m2*l1*l2*cos(theta2(i));
	Ac=m2*l2^2+m2*l1*l2*cos(theta2(i));
	Ad=m2*l2^2/rg2^2;
	%matric B
	Ba=0;
	Bb=-m2*l1*l2*sin(theta2(i));
	Bc=m2*l1*l2*sin(theta2(i));
	Bd=0;
	%matrix C
	Ca=-m2*l1*l2*sin(theta2(i));
	Cb=-m2*l1*l2*sin(theta2(i));
	Cc=0;
	Cd=0;
	%multiply by motion get h1 and h2
	h1=(Bb)*theta2Dot(i)^2+(Ca)*(theta1Dot(i)*theta2Dot(i))+(Cb)*(theta2Dot(i)*theta1Dot(i));
	h2=(Bc)*theta1Dot(i)^2;

	%torque limit check
	if u1(i)>tauL1
		taum1(i)=tauL1;
	else
		taum1(i)=u1(i);
	end

	if u2(i)>tauL2
		taum2(i)=tauL2;
	else
		taum2(i)=u2(i);
	end
	
	%friction check
	if theta1Dot(i)==0
		if taum1(i)-h1/rg1>tauc1
			tauf1=tauc1;
		elseif taum1(i)-h1/rg1<-tauc1
			tauf1=-tauc1;
		else
			tauf1=taum1(i)-h1/rg1;
		end
	
	else
		tauf1=tauc1*sign(theta1Dot(i));
	end
	
	if theta2Dot(i)==0
		if taum2(i)-h2/rg2>tauc2
			tauf2=tauc2;
		elseif taum2(i)-h2/rg2<-tauc2
			tauf2=-tauc2;
		else
			tauf2=taum2(i)-h2/rg2;
		end
	
	else
		tauf2=tauc2*sign(theta2Dot(i));
	end	
	
	
	%isolate thetaddot
	detM=1/(Aa*Ad-Ab*Ac);
	theta1DDot(i)=detM*(Ad*(rg1*(taum1(i)-tauf1))-Ab*(rg2*(taum2(i)-tauf2)));
	theta2DDot(i)=detM*(-Ac*(rg1*(taum1(i)-tauf1))+Aa*(rg2*(taum2(i)-tauf2)));
	%integration for angular position and angular velocity
	theta1Dot(i+1)=theta1Dot(i)+theta1DDot(i)*T;
	theta2Dot(i+1)=theta2Dot(i)+theta2DDot(i)*T;
	theta1(i+1)=theta1(i)+theta1Dot(i)*T;
	theta2(i+1)=theta2(i)+theta2Dot(i)*T;

    % friction movement check, not currently working
	%if sign(theta1DDot(i)*T+theta1Dot(i))~=sign(theta1Dot(i))&&theta1Dot(i)~=0
	%	theta1Dot(i+1)=0;
    %    theta1(i+1)=theta1(i);
	%end
	%if sign(theta2DDot(i)*T+theta2Dot(i))~=sign(theta2Dot(i))&&theta1Dot(i)~=0
	%	theta2Dot(i+1)=0;
    %   theta2(i+1)=theta2(i);
	%end

end
figure
subplot(3,2,1),plot(t,theta1DDot)
subplot(3,2,2),plot(t,theta1Dot)
subplot(3,2,3),plot(t,theta1)

figure
subplot(3,2,1),plot(t,theta2DDot)
subplot(3,2,2),plot(t,theta2Dot)
subplot(3,2,3),plot(t,theta2)
















