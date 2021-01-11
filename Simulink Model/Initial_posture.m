parameters

% Initial Values
xCr_init = 0;
yCr_init = 0;
phi_init = deg2rad(0);
psi_init = deg2rad(0);   

v_init = 1;


% Initial posture
% x1_init = 4;
% x2_init = 0;
% x3_init = 1;
% x4_init = 1;

% Posture at the end of t1
% x1_init = 0;
% x2_init = 0;
% x3_init = 1;
% x4_init = -3;

% Posture at the end of t1+T
% x1_init = 0;
% x2_init = 0;
% x3_init = 0;
% x4_init = -2.5;

%FOR ENCODER
px=1;
pp=1;
P=diag([px px pp]);
X=[xCr_init yCr_init phi_init]';   %X0


% FOR IMU
PI=diag([px px pp]);
XI=[xCr_init yCr_init phi_init]';   %X0

vx=v_init*cos(phi_init);
vy=v_init*sin(phi_init);

