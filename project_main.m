clc;
clear;

%variables to mess with
rg1=1; %gear ratio
rg2=1;
tauc1=250; %friction
tauc2=250;
tauL1=5000; %torque limit
tauL2=5000;
kp1=1000;	
kd1=600;	
kp2=1000;
kd2=300;
%noise=0.001;
encoderRes=2*pi/2^12;
beta1=0.6;	%constant for ILC
kilc1=50; %constant for ILC
beta2=0.6;	%constant for ILC
kilc2=50; %constant for ILC
in=10;		%ilteration number

ILC=true;
ADP=true;

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
theta1=zeros(in,N);  % position 1
theta1Dot=zeros(in,N); % speed 1
theta1DDot=zeros(in,N); % acceleration 1
theta2=zeros(in,N);  % position 2
theta2Dot=zeros(in,N); % speed 2
theta2DDot=zeros(in,N); % acceleration 2

theta1Des=zeros(1,N); % desired position 1
theta1DotDes=zeros(in,N); % desired speed 1
theta1DDotDes=zeros(in,N); % desired acceleration 1
theta2Des=zeros(1,N); % desired position 2
theta2DotDes=zeros(in,N); % desired speed 2
theta2DDotDes=zeros(in,N); % desired acceleration 2

theta1Sensed=zeros(in,N); % desired position 1
theta1DotSensed=zeros(in,N); % desired speed 1
theta1DDotSensed=zeros(in,N); % desired acceleration 1
theta2Sensed=zeros(in,N); % desired position 2
theta2DotSensed=zeros(in,N); % desired speed 2
theta2DDotSensed=zeros(in,N); % desired acceleration 2

u1=zeros(in,N);  % total control signal 1
u2=zeros(in,N);  % total control signal 2
uf1=zeros(in,N);  % feedback control signal 1
uf2=zeros(in,N);  % feedback control signal 2
ui1=zeros(in,N);  % ILC control signal 1
ui2=zeros(in,N);  % ILC control signal 2
ui1unfilt=zeros(in,N);
ui2unfilt=zeros(in,N);
ua1=zeros(in,N);  % adaptive control signal 1
ua2=zeros(in,N);  % adaptive control signal 2
err1=zeros(in,N);    % error
err2=zeros(in,N);

taum1=zeros(in,N);  % torque output 1
taum2=zeros(in,N);  % torque output 2

%target position
for i=1:N
theta1Des(i)=sin(0.005*i)*5;
theta2Des(i)=sin(-0.003*i)*5;

end





%construct robot dynamics

for n=1:in
	%ILC controller
	if n>1&&ILC==true

		ui1unfilt(n,:)=ui1(n,:)+beta1.*taum1(n-1,:)+kilc1.*err1(n-1,:);
		ui2unfilt(n,:)=ui2(n,:)+beta2.*taum2(n-1,:)+kilc2.*err2(n-1,:);
		[b,a] = butter(3,0.1);
		ui1(n,:)=filtfilt(b,a,ui1unfilt(n,:));
		ui2(n,:)=filtfilt(b,a,ui2unfilt(n,:));
	end
	for i=1:N-1
		

		%noise random
		%theta1Sensed(n,i)=theta1(n,i)+noise*randn;
		%theta2Sensed(n,i)=theta2(n,i)+noise*randn;
		%theta1DotSensed(n,i)=theta1Dot(n,i)+noise*randn;
		%theta2DotSensed(n,i)=theta2Dot(n,i)+noise*randn;
		
		%noise encoder res
		theta1Sensed(n,i)=round(theta1(n,i)/encoderRes)*encoderRes;
		theta2Sensed(n,i)=round(theta2(n,i)/encoderRes)*encoderRes;
		if i>1
			theta1DotSensed(n,i)=(theta1Sensed(n,i)-theta1Sensed(n,i-1))/T; 
			theta2DotSensed(n,i)=(theta2Sensed(n,i)-theta2Sensed(n,i-1))/T;
		end
		
		err1(n,i)=theta1Des(i)-theta1Sensed(n,i);
		err2(n,i)=theta2Des(i)-theta2Sensed(n,i);
		
		%PD controller
		if i==1

		uf1(n,i)=kp1*err1(n,i);
		uf2(n,i)=kp2*err2(n,i);
		else
		uf1(n,i)=kp1*err1(n,i)+kd1*(err1(n,i)-err1(n,i-1))/T;
		uf2(n,i)=kp2*err2(n,i)+kd2*(err2(n,i)-err2(n,i-1))/T;
		end
	


		%total control signal
		u1(n,i)=uf1(n,i)+ui1(n,i)+ua1(n,i);
		u2(n,i)=uf2(n,i)+ui2(n,i)+ua2(n,i);
     
  
        

		%matric A
		Aa=j1*rg1^2+(m1+m2)*l1^2+m2*l2^2+2*m2*l1*l2*cos(theta2(n,i));
		Ab=m2*l2^2+m2*l1*l2*cos(theta2(n,i));
		Ac=m2*l2^2+m2*l1*l2*cos(theta2(n,i));
		Ad=j2*rg2^2+m2*l2^2;
		M=[Aa Ab;Ac Ad];
		%matric B
		Ba=0;
		Bb=-m2*l1*l2*sin(theta2(n,i));
		Bc=m2*l1*l2*sin(theta2(n,i));
		Bd=0;
		B=[Ba Bb;Bc Bd];
		%matrix C
		Ca=-m2*l1*l2*sin(theta2(n,i));
		Cb=-m2*l1*l2*sin(theta2(n,i));
		Cc=0;
		Cd=0;
		C=[Ca Cb;Cc Cd];
		
		%multiply by motion get h1 and h2
		BB=[theta1Dot(n,i)^2;theta2Dot(n,i)^2];
		CC=[theta1Dot(n,i)*theta2Dot(n,i);theta2Dot(n,i)*theta1Dot(n,i)];
    
		H=B*BB+C*CC;

		%torque limit check
		if abs(u1(n,i))>tauL1
			taum1(n,i)=sign(u1(n,i))*tauL1;
		else
			taum1(n,i)=u1(n,i);
		end

		if abs(u2(n,i))>tauL2
			taum2(n,i)=sign(u2(n,i))*tauL2;
		else
			taum2(n,i)=u2(n,i);
		end
	
		%friction check
    
		if abs(theta1Dot(n,i))<0.0000000000001
			if taum1(n,i)-H(1)/rg1>tauc1
				tauf1=tauc1;
			elseif taum1(n,i)-H(1)/rg1<-tauc1
				tauf1=-tauc1;
			else
				tauf1=taum1(n,i)-H(1)/rg1;
			end
	
		else
       
			tauf1=tauc1*sign(theta1Dot(n,i));
		end
	
    
		%motor2
		if abs(theta2Dot(n,i))<0.0000000000001
			if taum2(n,i)-H(2)/rg2>tauc2
				tauf2=tauc2;
			elseif taum2(n,i)-H(2)/rg2<-tauc2
				tauf2=-tauc2;
			else
				tauf2=taum2(n,i)-H(2)/rg2;
			end
	
		else
			tauf2=tauc2*sign(theta2Dot(n,i));
		end	
	
	
	%isolate thetaddot
	
    Minv=inv(M);
	tau=[rg1*(taum1(n,i)-tauf1);rg2*(taum2(n,i)-tauf2)];
   
    acc=Minv*(tau-H);
    theta1DDot(n,i)=acc(1);
    theta2DDot(n,i)=acc(2); 
    
    
    %===================================
    %isolate thetaddot,det method
	%detM=1/(Aa*Ad-Ab*Ac);
	%theta1DDot(i)=detM*(Ad*(rg1*(taum1(i)-tauf1)-h1)-Ab*(rg2*(taum2(i)-tauf2)-h2));
    %theta1DDot(i)=0;
	%theta2DDot(i)=detM*(-Ac*(rg1*(taum1(i)-tauf1)-h1)+Aa*(rg2*(taum2(i)-tauf2)-h2));
    %===================================
    

    
   
    
	%probationary speed
	spd1=theta1Dot(n,i)+theta1DDot(n,i)*T;
	spd2=theta2Dot(n,i)+theta2DDot(n,i)*T;
    
    theta1Dot(n,i+1)=spd1;
    theta2Dot(n,i+1)=spd2;
 
    
    
	theta1(n,i+1)=theta1(n,i)+theta1Dot(n,i)*T;
	theta2(n,i+1)=theta2(n,i)+theta2Dot(n,i)*T;
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
subplot(5,2,8),plot(t,err1);
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










