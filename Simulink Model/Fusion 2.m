clear all
clc


dt=0.1
rw=0.060



X=[0 0 0 0 0 0 0 0 0.3 0.3 0 0 0 0 0 ]'
P=0.01*eye(15)

Q=zeros(15,1)
R=zeros(3,1)


vxe=[]
vye=[]
vxa=[]
vya=[]
vxf=[]
vyf=[]
H=[]

dtheta3=0 %from encoder
dtheta4=0  %from encoder

X_measure= zeros(15,1)

    for t=0:dt:10

b=       0.300    %while distance measured with error at t=0
Ax=      0.3       %measured with scale error from IMU at t=0
Ay=      0       %measured with scale error from IMU at t=0
Gz=      0      %measured with scale error from IMU at t=0
Bax= 0        %x Velocity measurement Bias
Bay= 0        %y Velocity measurement Bias
Bgz= 0        %Ang Velocity measurement Bias 
     

vl=rw*dtheta3;
vr=rw*dtheta4;
we=(vr-vl)/b   

ka=cos(we*dt)/2
kb=cos(we*dt)/2
kc=-0.5*(vl+vr)*dt*sin(we*dt)


kd=sin(we*dt)/2
ke=sin(we*dt)/2
kf=0.5*(vl+vr)*dt*cos(we*dt)




kg=1/b
kh=-1/b
ki=-(vr-vl)/(b^2)

A=zeros(15,15);
A(1,5)=kc;
A(1,7)=ka;
A(1,8)=kb;
A(2,5)=kf;
A(2,7)=kd;
A(2,8)=ke;
A(3,3)=1;
A(3,10)=dt;
A(3,11)=1;
A(4,4)=1;
A(4,12)=dt;
A(4,13)=1;
A(5,7)=kh;
A(5,8)=kg;
A(5,9)=ki;
A(6,14)=1;
A(6,15)=1;
A(7,7)=1;
A(8,8)=1;
A(9,9)=1;
A(10,10)=1;
A(11,11)=1;
A(12,12)=1;
A(13,13)=1;
A(14,14)=1;
A(15,15)=1;
A


C=zeros(3,15);
C(1,3)=-1;
C(1,5)=kc;
C(1,7)=ka;
C(1,8)=kb;
C(1,10)=-dt;
C(1,11)=-1;

C(2,4)=-1;
C(2,5)=kf;
C(2,7)=kd;
C(2,8)=ke;
C(2,12)=-dt;
C(2,13)=-1;


C(3,7)=kh;
C(3,8)=kg;
C(3,9)=ki;
C(3,14)=-1;
C(3,15)=-1;
C

K = P*C'/(C*P*C'+R)

X_measure=[0 0 0 0 0 0 vl vr b Ax Bax Ay Bay Gz Bgz ]'
Y         = C*X_measure + randn*R

X_measure_read=A*X_measure
    vxe = [vxe;X_measure_read(1)]
    vye = [vye;X_measure_read(2)]
    vxa = [vxa;X_measure_read(3)]
    vya = [vya;X_measure_read(4)]
    
%correct
T=(Y-C*X)

X=X+K(Y-C*X)
P = (eye(15)-K*C)*P


%Predict
    X = A*X
    P = A*P*A'+Q;
     
    vxf = [vxf;X(1)]
    vyf = [vyf;X(2)]
   
dtheta3= 0.5  +dtheta3    %measured with scale error from encoder at t=0
dtheta4= 0.5    +dtheta4  %measured with scale error from encoder at t=0


 end
t=0:0.1:10;
figure(1)
 plot(vxe,t,'r')
figure(2)
plot(vxa,t,'g')
figure(3)
  plot(vxf,t,'b')

figure(4)
  plot(H,t,'b')



