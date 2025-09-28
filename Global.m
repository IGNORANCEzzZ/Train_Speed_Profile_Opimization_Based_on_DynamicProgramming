%% Locomotive fixed parameters
global alpha_Re; % Regenerative braking utilization rate
alpha_Re=0;   
global Eta; % Locomotive efficiency
Eta=1;
global g; % Gravitational acceleration, N/kg
g=9.81; 
global Y; % Rotational mass coefficient
Y=0;
global M;
M=194; % Total train weight, t(k*kg)
global MaxSpeed;
MaxSpeed=80; % Maximum train speed, km/h
global Auxiliary_Power;
Auxiliary_Power=300.15; % Auxiliary power, kW

%% Maximum traction, braking force, and acceleration constraints
global Fd_max; % Maximum braking force, KN
Fd_max=166;
global Ft_max; % Maximum traction force, KN
Ft_max=205;
global a_max; % Maximum acceleration, m/s^2
a_max=1;
global a_min; % Minimum acceleration, m/s^2
a_min=-1;

%% Step size and spatial discrete points
global step_s; % Step size
step_s=1;
global startStation;
startStation=1;
global endStation;
endStation=2;
global Station % Station positions
Station = xlsread('03-线路参数','A1-A14车站');
global start_pos; % Starting position, unit: m
start_pos=Station(startStation);
global end_pos; % End position, unit: m
end_pos=Station(endStation);
global N; % Number of distance discrete points (x-axis)
N=ceil(abs(start_pos-end_pos)/step_s);

%% Speed precision and speed discrete points
global step_v;
step_v=0.01; % Speed discretization step, m/s

global Speed_N;
Speed_N=ceil(MaxSpeed/3.6/step_v);

%% Pre-loaded speed limit, gradient, and curve information
global SpeedLimit;
SpeedLimit=xlsread('03-线路参数','A1-A14限速');
global Gradient;
Gradient = xlsread('03-线路参数','A1-A14坡度');
global Curve;
 Curve = xlsread('03-线路参数','A1-A14曲线');

%% Boundary constraints
global v0;
v0=0;
global vend;
vend=0;
global Fe0;
Fe0=205; % Initial train traction force - braking force
global t0;
t0=0;

%% Punctuality constraints
global T; % Running time constraint, unit: seconds
T=110; 
global epsi_t; % Time constraint error tolerance
epsi_t=0.01*T;
% global lambda_T; % Time penalty factor
% lambda_T=3500;
global t_exp; % Expected running time for sub-interval, unit: s
v_average=abs(start_pos-end_pos)/T; % Average speed, m/s
% t_exp=step_s/v_average;
t_exp=0;

%% Pre-computed spatial discretized speed limits, track additional resistance, coordinate axis, maximum capacity curve
global wj; % Additional resistance, 1:N
[wj,~,~]=GetAddResistance();
global Dis_Space; % Spatial coordinates, 1:N+1
global MaxCapacityV; % Maximum capacity speeds, 1:N+1
[Dis_Space,MaxCapacityV]=MaxCapacityCurve(1);