function [J_k,t_k,F_k,e_k]=RecurrenceEquation(v_k,v_k_1,k)
%% 输入：
%v_k：前一个离散点的速度，单位m/s
%v_k_1:后一个离散点的速度，单位m/s
%k，空间域第k个点，以起车点为第一个点
%% 输出：
% J_k：从k点转移到k+1点所需要的代价
% t_k：从k点转移到k+1点所需要的运行时间，单位s
% F_k：从k点转移到k+1点所使用的力（牵引或制动力），单位N
% e_k；从k点转移到k+1点所使用的考虑再生制动的能耗，单位焦耳，J
%%
global step_s;
global M;
global g;
global wj;
global t_exp;
global lambda_T;
global Eta;
global alpha_Re;
global MaxCapacityV;%1:N+1,最大能力速度曲线
global a_max;
global a_min;

a=(v_k_1^2-v_k^2)/(2*step_s);%单位m/s^2
w0=GetBasicResistance(v_k*3.6)*M*g;%单位N/KN * KKG/ N/KG=N
w_j=wj(1,k)*1000;%单位KN*1000=N；
F_k=a*M*1000+w0+w_j;
if v_k+v_k_1==0
    t_k=inf;
else
    t_k=2*step_s/(v_k+v_k_1);%使用平均速度求t_k
end
Ft_max=GetTractionForce(v_k*3.6)*1000;%此速度下的最大牵引力，单位N
Fd_max=GetMaxBrakeForce(v_k*3.6)*1000;%此速度下的最大制动力，单位N
x=['a= ',num2str(a),' F_k= ',num2str(F_k),' w0= ',num2str(w0),' w_j= ',num2str(w_j),' t_k= ',num2str(t_k),' MaxCapacityV(1,k)=',num2str(MaxCapacityV(1,k)),' MaxCapacityV(1,k+1)=',num2str(MaxCapacityV(1,k+1))];
global N;
if k==N
disp(x)
end
%% 判断输入的两个速度之间的状态转移是否符合 加速度约束，最大牵引、制动力约束，以及最大能力曲线约束
if a>=a_min && a<= a_max && F_k>=-Fd_max && F_k<=Ft_max && v_k*3.6<=MaxCapacityV(1,k) && v_k_1*3.6<=MaxCapacityV(1,k+1)
% if a>=a_min && a<= a_max && F_k>=-Fd_max && F_k<=Ft_max
    if F_k>=0
        J_k=F_k/Eta*step_s+lambda_T*abs(t_k-t_exp);
    else
        J_k=-alpha_Re*Eta*F_k*step_s+lambda_T*abs(t_k-t_exp);
    end
else
    J_k=inf;
end

if F_k>=0
    e_k=F_k/Eta*step_s;
else
    e_k=-alpha_Re*Eta*F_k*step_s;
end