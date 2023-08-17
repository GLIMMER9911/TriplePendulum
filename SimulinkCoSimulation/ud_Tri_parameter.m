%% UnderdrivernTriplePendulumMotor Parameters
PI = 3.141592653589793;
w = 2*PI;
%% Link1 data
L = 0.4;
H = 0.05;
W = 0.05;

link1_m = 2;
link1_CM = [0  0  0];
link1_MI = [8.333333333338782e-04 0.0271 0.0271];
link1_PI = [0 0 0];



%% Link2 data

link2_m = 2;
link2_CM = [0  0  0];
link2_MI = [8.333333333338782e-04 0.0271 0.0271];
link2_PI = [0 0 0];

%% Link3 data

link3_m = 2;
link3_CM = [0  0  0];
link3_MI = [8.333333333338782e-04 0.0271 0.0271];
link3_PI = [0 0 0 ];

%% Electric motor parameters

max_torque = 80;

motor_voltage = 24;
motor_resistance = 0.200;
motor_constant = 0.011;
motor_inertia = 1e-5;
motor_damping = 1e-5;
motor_inductance = 200e-6;
motor_time_constant = 1.e-03;

gear_ratio = 9;

% time = 0:0.01:10;
% time = time';
% ssize = size(time);
% siminLink = zeros(ssize(1),4);

%% PID Parameter
% Input x,y, using PD to control 

X_KP = 100;
X_KI = 0.1;
X_KD = 0;


Y_KP = 100;
Y_KI = 0.1;
Y_KD = 0;

% Input end effector, using PID to control 





















