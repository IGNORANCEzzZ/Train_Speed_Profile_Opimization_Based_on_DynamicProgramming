function [J_k,t_k,F_k,e_k]=RecurrenceEquation(v_k,v_k_1,k)
%% Inputs:
% v_k: Speed at previous discrete point, unit: m/s
% v_k_1: Speed at next discrete point, unit: m/s
% k: Spatial point k, with train starting point as first point
%% Outputs:
% J_k: Cost required for transition from point k to k+1
% t_k: Running time required for transition from point k to k+1, unit: s
% F_k: Traction force used for transition from point k to k+1, unit: N
% e_k: Energy consumption used for transition from point k to k+1 (traction energy), unit: J
%%
global step_s;
global M;
global g;
global wj;
global t_exp;
global lambda_T;
global Eta;
global alpha_Re;
global MaxCapacityV; % Maximum capacity speeds, 1:N+1
global a_max;
global a_min;

a=(v_k_1^2-v_k^2)/(2*step_s); % Acceleration, unit: m/s^2
w0=GetBasicResistance(v_k*3.6)*M*g; % Basic resistance, unit: N/KN * KKG / N/KG = N
w_j=wj(1,k)*1000; % Additional resistance, unit: KN*1000 = N
F_k=a*M*1000+w0+w_j;
if v_k+v_k_1==0
    t_k=inf;
else
    t_k=2*step_s/(v_k+v_k_1); % Using average speed for t_k
end
Ft_max=GetTractionForce(v_k*3.6)*1000; % Maximum traction force at current speed, unit: N
Fd_max=GetMaxBrakeForce(v_k*3.6)*1000; % Maximum braking force at current speed, unit: N
x=['a= ',num2str(a),' F_k= ',num2str(F_k),' w0= ',num2str(w0),' w_j= ',num2str(w_j),' t_k= ',num2str(t_k),' MaxCapacityV(1,k)=',num2str(MaxCapacityV(1,k)),' MaxCapacityV(1,k+1)=',num2str(MaxCapacityV(1,k+1))];
global N;
if k==N
disp(x)
end

%% Check if state transition between two speeds is feasible: speed constraints, traction/braking force constraints, acceleration constraints
if a>=a_min && a<= a_max && F_k>=-Fd_max && F_k<=Ft_max && v_k*3.6<=MaxCapacityV(1,k) && v_k_1*3.6<=MaxCapacityV(1,k+1)
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