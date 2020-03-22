clc;
clear;

%variables to mess with
rg1=100; %gear ratio
rg2=100;
tauc1=10; %friction
tauc2=10;
tauL1=1000; %torque limit
tauL2=1000;
noise=0;
kp1=1;	
kd1=0.00005;	
kp2=0;
kd2=0;
randn('seed', 8);




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
N=10000; % number of points
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

theta1Sensed=zeros(1,N); % desired position 1
theta1DotSensed=zeros(1,N); % desired speed 1
theta1DDotSensed=zeros(1,N); % desired acceleration 1
theta2Sensed=zeros(1,N); % desired position 2
theta2DotSensed=zeros(1,N); % desired speed 2
theta2DDotSensed=zeros(1,N); % desired acceleration 2

u1=zeros(1,N);  % total control signal 1
u2=zeros(1,N);  % total control signal 2
taum1=zeros(1,N);  % torque output 1
taum2=zeros(1,N);  % torque output 2

%target position
for i=1:5000
theta1Des(i)=100;
theta2Des(i)=0;

end
for i=5000:N-1
theta1Des(i)=50;
theta2Des(i)=0;

end




%construct robot dynamics

for i=1:N-1
    
    %input open loop testing
    %u1(i)=0;  % torque output 1
	%u2(i)=1;  % torque output 2
	
	%noise
	theta1Sensed(i)=theta1(i)+noise*randn;
	theta2Sensed(i)=theta1(i)+noise*randn;
	
	%prepare for d controller
	theta1DotDes(i)=(theta1Des(i)-theta1Sensed(i))/T;
	theta2DotDes(i)=(theta2Des(i)-theta2Sensed(i))/T;	
	
	%PD controller
	u1(i)=kp1*(theta1Des(i)-theta1Sensed(i))+kd1*(theta1DotDes(i)-theta1DotSensed(i));
	u2(i)=kp2*(theta2Des(i)-theta2Sensed(i))+kd2*(theta2DotDes(i)-theta2DotSensed(i));
	

	%matric A
	Aa=j1*rg1^2+(m1+m2)*l1^2+m2*l2^2+2*m2*l1*l2*cos(theta2(i));
	Ab=m2*l2^2+m2*l1*l2*cos(theta2(i));
	Ac=m2*l2^2+m2*l1*l2*cos(theta2(i));
	Ad=j2*rg2^2+m2*l2^2;
    M=[Aa Ab;Ac Ad];
	%matric B
	Ba=0;
	Bb=-m2*l1*l2*sin(theta2(i));
	Bc=m2*l1*l2*sin(theta2(i));
	Bd=0;
    B=[Ba Bb;Bc Bd];
	%matrix C
	Ca=-m2*l1*l2*sin(theta2(i));
	Cb=-m2*l1*l2*sin(theta2(i));
	Cc=0;
	Cd=0;
    C=[Ca Cb;Cc Cd];
    
	%multiply by motion get h1 and h2
    BB=[theta1Dot(i)^2;theta2Dot(i)^2];
    CC=[theta1Dot(i)*theta2Dot(i);theta2Dot(i)*theta1Dot(i)];
    
    H=B*BB+C*CC;

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
		if taum1(i)-H(1)/rg1>tauc1
			tauf1=tauc1;
		elseif taum1(i)-H(1)/rg1<-tauc1
			tauf1=-tauc1;
		else
			tauf1=taum1(i)-H(1)/rg1;
		end
	
    else
       
        tauf1=tauc1*sign(theta1Dot(i));
    end
	
    
    %motor2
	if theta2Dot(i)==0
		if taum2(i)-H(2)/rg2>tauc2
			tauf2=tauc2;
		elseif taum2(i)-H(2)/rg2<-tauc2
			tauf2=-tauc2;
		else
			tauf2=taum2(i)-H(2)/rg2;
		end
	
	else
		tauf2=tauc2*sign(theta2Dot(i));
	end	
	
	
	%isolate thetaddot
	
    Minv=inv(M);
	tau=[rg1*(taum1(i)-tauf1);rg2*(taum2(i)-tauf2)];
   
    acc=Minv*(tau-H);
    theta1DDot(i)=acc(1);
    theta2DDot(i)=acc(2); 
    
    
    %===================================
    %isolate thetaddot,det method
	%detM=1/(Aa*Ad-Ab*Ac);
	%theta1DDot(i)=detM*(Ad*(rg1*(taum1(i)-tauf1)-h1)-Ab*(rg2*(taum2(i)-tauf2)-h2));
    %theta1DDot(i)=0;
	%theta2DDot(i)=detM*(-Ac*(rg1*(taum1(i)-tauf1)-h1)+Aa*(rg2*(taum2(i)-tauf2)-h2));
    %===================================
    

    
   
    
	%probationary speed
	spd1=theta1Dot(i)+theta1DDot(i)*T;
	spd2=theta2Dot(i)+theta2DDot(i)*T;
    
    %trying to make a mechanism to stop friction over shoot
    %if sign(spd1)~= sign(theta1Dot(i)) && abs(theta1Dot(i))>0.001
    %    theta1Dot(i+1)=0;
    %else
        theta1Dot(i+1)=spd1;
    %end
    
    %if sign(spd2)~= sign(theta2Dot(i)) && abs(theta2Dot(i))>0.001
    %    theta2Dot(i+1)=0;
    %else
        theta2Dot(i+1)=spd2;
    %end    
    
    
	theta1(i+1)=theta1(i)+theta1Dot(i)*T;
	theta2(i+1)=theta2(i)+theta2Dot(i)*T;



end
figure
subplot(4,2,1),plot(t,u1)
subplot(4,2,2),plot(t,theta1DDot)
subplot(4,2,3),plot(t,theta1Dot)
subplot(4,2,4),plot(t,theta1)



figure
subplot(4,2,1),plot(t,u2)
subplot(4,2,2),plot(t,theta2DDot)
subplot(4,2,3),plot(t,theta2Dot)
subplot(4,2,4),plot(t,theta2)
















