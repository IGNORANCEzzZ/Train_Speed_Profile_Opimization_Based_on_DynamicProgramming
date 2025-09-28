%% �����̶�����
global alpha_Re; %�����ƶ�������
alpha_Re=0.7;
global Eta;%����Ч��
Eta=0.9;
global g;%N/kg
g=9.8; 
global Y;%��ת����ϵ��
Y=0.05;
global M;
M=194.295; %�г����أ�kg
global MaxSpeed;
MaxSpeed=80; %�г�����ٶȣ�km/h

%% ���ǣ�����ƶ��������ٶ�Լ��
global Fd_max; %��λKN
Fd_max=166;
global Ft_max; %��λKN
Ft_max=203;
global a_max;%��λm/s^2
a_max=1;
global a_min;%��λm/s^2
a_min=-1;

%% �������ռ���ɢ����
global step_s;%����
step_s=10;
global startStation;
startStation=6;
global endStation;
endStation=8;
global Station %��վλ��
Station = xlsread('03-��·����','A1-A14��վ');
global start_pos;%��ʼ��λ�ã���λm
start_pos=Station(startStation);
global end_pos;%������λ�ã���λm
end_pos=Station(endStation);
global N;%x�ᣬ������ɢ����
N=ceil(abs(start_pos-end_pos)/step_s);

%% �ٶȾ��ȡ��ٶ���ɢ����
global step_v;
step_v=0.1;%m/s

global Speed_N;
Speed_N=ceil(MaxSpeed/3.6/step_v);

%% Ԥ�ȶ�ȡ�����١��¶ȡ�������Ϣ
global SpeedLimit;
SpeedLimit=xlsread('03-��·����','A1-A14����');
global Gradient;
Gradient = xlsread('03-��·����','A1-A14�¶�');
global Curve;
 Curve = xlsread('03-��·����','A1-A14����');

%% �˵�Լ��
global v0;
v0=0;
global vend;
vend=0;
global Fe0;
Fe0=203;%��ʼ���г�ǣ����-�ƶ���???
global t0;
t0=0;

%% ׼��Լ��
global T;%����ʱ��Լ������λ��
T=220; 
global epsi_t; %ʱ��Լ�������
epsi_t=0.99*T;
global lambda_T;%ʱ��ͷ�����
lambda_T=1e8;
global t_exp; %��������������ʱ�䣬��λs
v_average=abs(start_pos-end_pos)/T;%ƽ���ٶȣ�m/s
t_exp=step_s/v_average;

%% Ԥ�ȵõ��Ŀռ���ɢ�������١���·���������������ᣬ�����������
global wj;% 1:N
[wj,~,~]=GetAddResistance();
global Dis_Space;% 1:N+1
global MaxCapacityV;%1:N+1
[Dis_Space,MaxCapacityV]=MaxCapacityCurve(1);