%% 机车固定参数
global alpha_Re; %再生制动利用率
alpha_Re=0;   
global Eta;%机车效率
Eta=1;
global g;%N/kg
g=9.81; 
global Y;%回转质量系数
Y=0;
global M;
M=194; %列车总重，t(k*kg)
global MaxSpeed;
MaxSpeed=80; %列车最大速度，km/h
global Auxiliary_Power;
Auxiliary_Power=300.15;%kw
%% 最大牵引、制动力、加速度约束
global Fd_max; %单位KN
Fd_max=166;
global Ft_max; %单位KN
Ft_max=205;
global a_max;%单位m/s^2
a_max=1;
global a_min;%单位m/s^2
a_min=-1;
%% 步长、空间离散点数
global step_s;%步长
step_s=1;
global startStation;
startStation=1;
global endStation;
endStation=2;
global Station%车站位置
Station = xlsread('03-线路参数','A1-A14车站');
global start_pos;%开始点位置，单位m
start_pos=Station(startStation);
global end_pos;%结束点位置，单位m
end_pos=Station(endStation);
global N;%x轴，距离离散点数
N=ceil(abs(start_pos-end_pos)/step_s);

%% 速度精度、速度离散点数
global step_v;
step_v=0.01;%m/s

global Speed_N;
Speed_N=ceil(MaxSpeed/3.6/step_v);

%% 预先读取的限速、坡度、曲线信息
global SpeedLimit;
SpeedLimit=xlsread('03-线路参数','A1-A14限速');
global Gradient;
Gradient = xlsread('03-线路参数','A1-A14坡度');
global Curve;
 Curve = xlsread('03-线路参数','A1-A14曲线');

%% 端点约束
global v0;
v0=0;
global vend;
vend=0;
global Fe0;
Fe0=205;%初始的列车牵引力-制动力???
global t0;
t0=0;

%% 准点约束
global T;%运行时间约束，单位秒
T=110; 
global epsi_t; %时间约束的误差
epsi_t=0.01*T;
% global lambda_T;%时间惩罚因子
% lambda_T=3500;
global t_exp; %子区间期望运行时间，单位s
v_average=abs(start_pos-end_pos)/T;%平均速度，m/s
%t_exp=step_s/v_average;
t_exp=0;
%% 预先得到的空间离散化的限速、线路附加阻力、坐标轴，最大能力曲线
global wj;% 1:N
[wj,~,~]=GetAddResistance();
global Dis_Space;% 1:N+1
global MaxCapacityV;%1:N+1
[Dis_Space,MaxCapacityV]=MaxCapacityCurve(1);