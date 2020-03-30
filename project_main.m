clc;
clear;

%variables to mess with
rg1=1; %gear ratio
rg2=1;
tauc1=250; %friction
tauc2=250;
tauL1=2500; %torque limit
tauL2=2500;
kp1=500;	
kd1=500;	
kp2=500;
kd2=500;
noise=0.001;
%beta=1;	%constant for ILC
kilc=0.1; %constant for ILC
in=100;		%ilteration nymber

randn('seed', 8);



%mass&length
m1=10;
m2=5;
l1=1;
l2=1;

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
uf1=zeros(1,N);  % feedback control signal 1
uf2=zeros(1,N);  % feedback control signal 2
ui1=zeros(1,N);  % ILC control signal 1
ui2=zeros(1,N);  % ILC control signal 2
ua1=zeros(1,N);  % adaptive control signal 1
ua2=zeros(1,N);  % adaptive control signal 2
err1=zeros(1,N);    % error
err2=zeros(1,N);

taum1=zeros(1,N);  % torque output 1
taum2=zeros(1,N);  % torque output 2

%target position
for i=1:N
theta1Des(i)=sin(0.005*i)*5;
theta2Des(i)=sin(-0.005*i)*5;

end





%construct robot dynamics

for n=1:in
	%ILC controller
    ui1unfilt=ui1+kilc.*err1;
	ui2unfilt=ui2+kilc.*err2;
    %ui1unfilt=ui1+beta.*u1;
	%ui2unfilt=ui2+beta.*u1;
	ui1=filtfilt([1 3 3 1]/6,[3 0 1 0]/3,uf1);
	ui2=filtfilt([1 3 3 1]/6,[3 0 1 0]/3,uf2);
	for i=1:N-1
		

		%noise
		theta1Sensed(i)=theta1(i)+noise*randn;
		theta2Sensed(i)=theta2(i)+noise*randn;
		theta1DotSensed(i)=theta1Dot(i)+noise*randn;
		theta2DotSensed(i)=theta2Dot(i)+noise*randn;
		
		err1(i)=theta1Des(i)-theta1Sensed(i);
		err2(i)=theta2Des(i)-theta2Sensed(i);
		%PD controller
		if i==1

		uf1(i)=kp1*err1(i);
		uf2(i)=kp2*err2(i);
		else
		uf1(i)=kp1*err1(i)+kd1*(err1(i)-err1(i-1))/T;
		uf2(i)=kp2*err2(i)+kd2*(err2(i)-err2(i-1))/T;
		end
	


		%total control signal
		u1(i)=uf1(i)+ui1(i)+ua1(i);
		u2(i)=uf2(i)+ui2(i)+ua2(i);
     
  
        

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
		if abs(u1(i))>tauL1
			taum1(i)=sign(u1(i))*tauL1;
		else
			taum1(i)=u1(i);
		end

		if abs(u2(i))>tauL2
			taum2(i)=sign(u2(i))*tauL2;
		else
			taum2(i)=u2(i);
		end
	
		%friction check
    
		if abs(theta1Dot(i))<0.0000000000001
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
		if abs(theta2Dot(i))<0.0000000000001
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
    
    theta1Dot(i+1)=spd1;
    theta2Dot(i+1)=spd2;
 
    
    
	theta1(i+1)=theta1(i)+theta1Dot(i)*T;
	theta2(i+1)=theta2(i)+theta2Dot(i)*T;
	end


end

figure
subplot(5,2,1),plot(t,taum1)
title('motor1 input');
subplot(5,2,2),plot(t,uf1)
title('motor1 PD');
subplot(5,2,3),plot(t,ui1)
title('motor1 ILC');
subplot(5,2,4),plot(t,ua1)
title('motor1 Adp');
subplot(5,2,5),plot(t,theta1DDot)
title('motor1 acc');
subplot(5,2,6),plot(t,theta1Dot)
title('motor1 spd');
subplot(5,2,7),plot(t,theta1,t,theta1Des)
title('motor1 Setpoint&Output');
subplot(5,2,8),plot(t,err2);
title('motor1 err');



figure
subplot(5,2,1),plot(t,taum2)
title('motor2 input');
subplot(5,2,2),plot(t,uf2)
title('motor2 PD');
subplot(5,2,3),plot(t,ui2)
title('motor2 ILC');
subplot(5,2,4),plot(t,ua2)
title('motor2 Adp');
subplot(5,2,5),plot(t,theta2DDot)
title('motor2 acc');
subplot(5,2,6),plot(t,theta2Dot)
title('motor2 spd');
subplot(5,2,7),plot(t,theta2,t,theta2Des)
title('motor2 Setpoint&Output');
subplot(5,2,8),plot(t,err2);
title('motor2 err');















