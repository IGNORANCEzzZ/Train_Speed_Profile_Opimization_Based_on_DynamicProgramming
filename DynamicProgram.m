function [s,v,F,T,E,Matrix_Jmin,Et,Eb,T1]=DynamicProgram(lambda)

%% 输出
% s: 离散的空间轴，单位m
% v：优化后的速度曲线，单位km/h
% F: 列车输出力在空间域上的离散序列，单位N
% T：列车总运行时间，单位s
% E：考虑再生制动的列车总牵引能耗,单位J

%% 自底而上的常规动态规划求解方法
global Speed_N;
global N;
global step_s;
global step_v;
global v0;
global start_pos;
global end_pos;
global MaxCapacityV;%1:N+1,最大能力速度曲线

global wj;
global t_exp;
global Eta;
global alpha_Re;
global a_max;
global a_min;
global M;
global g;

lambda_T=lambda;

if end_pos>start_pos
    step=step_s;
else
   step=-step_s;
end

Matrix_Jmin=inf*ones(Speed_N+1,N);%回溯记录矩阵 之 状态转移代价累计矩阵
%第i行表示(i-1)Δv的速度，第j列表示空间域上第j个点转移到终点（第N+1个点）所需要的最小的总代价Jmin
%（i,j）表示从空间域上第j个点，以速度(i-1)Δv转移到终点（第N+1个点）所需要的最小的总代价Jmin
%j=1:N，j=1表示从起始站转移到起始站后一个点
Matrix_Select=ones(Speed_N+1,N);%回溯记录矩阵 之 状态转移最优路径选择矩阵

%% 终点站前一个点转移到终点站
v_N=zeros(Speed_N+1,1);
v_N_1=zeros(Speed_N+1,1);
for i=1:1:Speed_N+1
    vi=step_v*(i-1);
    v_N(i,1)=vi;
    v_N_1(i,1)=0;
end
a_N=(v_N<=MaxCapacityV(N) & v_N_1<=MaxCapacityV(N+1)).*((v_N_1+v_N).*(v_N_1-v_N)/(2*step_s))+(v_N>MaxCapacityV(N) | v_N_1>MaxCapacityV(N+1)).*(1e16);%加速度矩阵
R_N=GetBasicResistance(v_N*3.6)*M*g + wj(1,N)*1000;%基本运行阻力和线路附加阻力矩阵
Ft_max_N=GetTractionForce(v_N*3.6)*1000;%最大牵引力矩阵，单位N
Fd_max_N=GetMaxBrakeForce(v_N*3.6)*1000;%最大制动力矩阵，单位N
F_N=(a_N>=a_min & a_N<=a_max).*(a_N*M*1000+R_N)+(a_N>a_max |a_N<a_min).*(1e16);
Cost_J_N=(F_N>Ft_max_N|F_N<-Fd_max_N).*(1e16)+(F_N>=0 & F_N<=Ft_max_N).*(F_N*step_s/Eta)+(F_N<0 & F_N>=-Fd_max_N).*(F_N*step_s*alpha_Re*Eta);
t_N=2*step_s./(v_N+v_N_1);
%时间矩阵
Cost_N=lambda_T*abs(t_N-t_exp)+Cost_J_N;
Matrix_Jmin(:,N)=Cost_N;
Matrix_Select(:,N)=1;

%% 起点站后第一个点转移到终点站前一个点
for j=N-1:-1:2%从N-1次状态转移到第二次状态转移（起点站后一个点转移到起点站后两个点）
%     
%      x=['列= ',num2str(j)];
%      disp(x)
     
    v_j=zeros(Speed_N+1,Speed_N+1);
    v_j_1=zeros(Speed_N+1,Speed_N+1);
    for i=1:1:Speed_N+1
        vi=step_v*(i-1)*ones(1,Speed_N+1);
        v_j(i,:)=vi;
        v_j_1(i,:)=vi;
    end
    v_j_1=v_j_1';%转置
    
    aj_j_1=(v_j<=MaxCapacityV(j) & v_j_1<=MaxCapacityV(j+1)).*((v_j_1+v_j).*(v_j_1-v_j)/(2*step_s))+(v_j>MaxCapacityV(j) | v_j_1>MaxCapacityV(j+1)).*(1e16);%加速度矩阵
    %状态j中所有可能的速度点转移到j+1状态所有可能的速度点的加速度的矩阵
    %a(m,n)表示j状态速度为step_v*(m-1) 转移到 j+1状态速度为tep_v*(n-1)所需要的加速度
    
    R=GetBasicResistance(v_j*3.6)*M*g + wj(1,j)*1000;%基本运行阻力和线路附加阻力矩阵
    Ft_max=GetTractionForce(v_j*3.6)*1000;%最大牵引力矩阵，单位N
    Fd_max=GetMaxBrakeForce(v_j*3.6)*1000;%最大制动力矩阵，单位N
    
    F_k=(aj_j_1>=a_min & aj_j_1<=a_max).*(aj_j_1*M*1000+R)+(aj_j_1>a_max |aj_j_1<a_min).*(1e32);
    %状态j中所有可能的速度点转移到j+1状态所有可能的速度点 的 列车施加力的 矩阵
    
    Cost_J=(F_k>Ft_max|F_k<-Fd_max).*(1e32)+(F_k>=0 & F_k<=Ft_max).*(F_k*step_s/Eta)+(F_k<0 & F_k>=-Fd_max).*(F_k*step_s*alpha_Re*Eta);
    % 牵引能耗代价矩阵
    
    t_k=2*step_s./(v_j+v_j_1);
    %时间矩阵
    Cost=lambda_T*abs(t_k-t_exp)+Cost_J;
    CostT=Cost'+Matrix_Jmin(:,j+1);
    [value,index]=min(CostT);
    Matrix_Jmin(:,j)=value';
    Matrix_Select(:,j)=index';
    
end
%% 从起点站转移到起点站后第一个点
v_1=zeros(Speed_N+1,1);
v_2=zeros(Speed_N+1,1);
for i=1:1:Speed_N+1
    vi=step_v*(i-1);
    v_1(i,1)=0;
    v_2(i,1)=vi;
end
a_1=(v_1<=MaxCapacityV(1) & v_2<=MaxCapacityV(2)).*((v_2+v_1).*(v_2-v_1)/(2*step_s))+(v_1>MaxCapacityV(1) | v_2>MaxCapacityV(2)).*(1e16);%加速度矩阵
R_1=GetBasicResistance(v_1*3.6)*M*g + wj(1,1)*1000;%基本运行阻力和线路附加阻力矩阵
Ft_max_1=GetTractionForce(v_1*3.6)*1000;%最大牵引力矩阵，单位N
Fd_max_1=GetMaxBrakeForce(v_1*3.6)*1000;%最大制动力矩阵，单位N
F_1=(a_1>=a_min & a_1<=a_max).*(a_1*M*1000+R_1)+(a_1>a_max |a_1<a_min).*(1e16);
Cost_J_1=(F_1>Ft_max_1|F_1<-Fd_max_1).*(1e16)+(F_1>=0 & F_1<=Ft_max_1).*(F_1*step_s/Eta)+(F_1<0 & F_1>=-Fd_max_1).*(F_1*step_s*alpha_Re*Eta);
t_1=2*step_s./(v_1+v_2);
%时间矩阵
Cost_1=lambda_T*abs(t_1-t_exp)+Cost_J_1;
Matrix_Jmin(:,1)=Cost_1;
Matrix_Select(:,1)=1:1:Speed_N+1;

%% 搜寻第一列中总转移代价最小的点，然后通过Matrix_Select矩阵正向连结最优转移路径上的所有速度点


[value,flag]=min(Matrix_Jmin(:,1));

s=zeros(1,N+1);%起始站点作为矩阵第一个点
v=zeros(1,N+1);%起始站点作为矩阵第一个点
F=zeros(1,N+1);%起始站点作为矩阵第一个点
s(1,1)=start_pos;
v(1,1)=v0;
F(1,N+1)=0;
T=0;
E=0;
Et=0;
Eb=0;
T1=0;
for i=1:1:N
    s(1,i+1)=s(1,i)+step;
    flag=Matrix_Select(flag,i);
    v(1,i+1)=step_v*(flag-1);
    if s(1,i+1)>=12240
       T1=T1+2*step_s./(v(1,i)+v(1,i+1));
    end
    T=T+2*step_s./(v(1,i)+v(1,i+1));
    a=(v(1,i+1)^2-v(1,i)^2)/(2*step_s);
    F(1,i)=a*M*1000+GetBasicResistance(v(1,i)*3.6)*M*g + wj(1,i)*1000;
    E=E+( F(1,i)>=0).*(F(1,i)*step_s/Eta)+(F(1,i)<0).*(F(1,i)*alpha_Re*Eta*step_s);
    Et=Et+( F(1,i)>=0).*(F(1,i)*step_s/Eta)+(F(1,i)<0).*(0);
    Eb=Eb+( F(1,i)>=0).*(0)+(F(1,i)<0).*(F(1,i)*alpha_Re*Eta*step_s);
end
t=toc;
x=['时间已过',num2str(t), ' s'];
disp(x)
end
