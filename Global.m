%% Vehicle dynamics parameters
global alpha_Re; % Regenerative braking absorption rate
alpha_Re=0.7;
global Eta; % Traction efficiency
Eta=0.9;
global g; % Gravitational acceleration, N/kg
g=9.8; 
global Y; % Rotational inertia coefficient
Y=0.05;
global M;
M=194.295; % Train mass, kg
global MaxSpeed;
MaxSpeed=80; % Maximum train speed, km/h

%% Train forces: traction and braking, speed constraints
global Fd_max; % Maximum braking force, KN
Fd_max=166;
global Ft_max; % Maximum traction force, KN
Ft_max=203;
global a_max; % Maximum acceleration, m/s^2
a_max=1;
global a_min; % Minimum acceleration (deceleration), m/s^2
a_min=-1;

%% Spatial domain discretization
global step_s; % Step size
step_s=10;
global startStation;
startStation=6;
global endStation;
endStation=8;
global Station % Station positions
Station = xlsread('03-线路数据','A1-A14车站');
global start_pos; % Starting position, unit: m
start_pos=Station(startStation);
global end_pos; % End position, unit: m
end_pos=Station(endStation);
global N; % Number of discrete points in space (x-axis)
N=ceil(abs(start_pos-end_pos)/step_s);

%% Speed axis values and speed discretization
global step_v;
step_v=0.1; % Speed discretization step, m/s

global Speed_N;
Speed_N=ceil(MaxSpeed/3.6/step_v);

%% Pre-load speed limit, gradient, and curve information
global SpeedLimit;
SpeedLimit=xlsread('03-线路数据','A1-A14限速');
global Gradient;
Gradient = xlsread('03-线路数据','A1-A14坡度');
global Curve;
 Curve = xlsread('03-线路数据','A1-A14曲线');

%% Boundary constraints
global v0;
v0=0;
global vend;
vend=0;
global Fe0;
Fe0=203; % Initial train force: traction-braking force
global t0;
t0=0;

%% Punctuality constraints
global T; % Running time constraint, unit: seconds
T=220; 
global epsi_t; % Time constraint softening coefficient
epsi_t=0.99*T;
global lambda_T; % Time penalty coefficient
lambda_T=1e8;
global t_exp; % Expected running time for each step, unit: s
v_average=abs(start_pos-end_pos)/T; % Average speed, m/s
t_exp=step_s/v_average;

%% Pre-compute spatial discrete coordinates, gradients, and track curve information to reduce computational complexity and ensure efficiency
global wj; % Additional resistance, 1:N
[wj,~,~]=GetAddResistance();
global Dis_Space; % Discrete space coordinates, 1:N+1
global MaxCapacityV; % Maximum capacity speeds, 1:N+1
[Dis_Space,MaxCapacityV]=MaxCapacityCurve(1);