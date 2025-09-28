function [s,v,F,T,E,Matrix_Jmin,Et,Eb,T1]=DynamicProgram(lambda)

%% Outputs:
% s: Discrete spatial coordinates, unit: m
% v: Optimized speed profile, unit: km/h
% F: Train forces at each spatial discrete point, unit: N
% T: Train running time, unit: s
% E: Total energy consumption (traction energy), unit: J

%% Dynamic programming solution for energy-optimal train control
global Speed_N;
global N;
global step_s;
global step_v;
global v0;
global start_pos;
global end_pos;
global MaxCapacityV; % Maximum capacity speeds, 1:N+1

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

Matrix_Jmin=inf*ones(Speed_N+1,N); % Data structure to record minimum cumulative cost of state transitions
% Row i represents speed (i-1)*step_v, column j represents transition from spatial point j to destination (N+1 point), recording minimum cumulative cost Jmin
% Element (i,j) represents minimum cumulative cost Jmin from spatial point j with speed (i-1)*step_v to destination (N+1 point)
% j=1:N, j=1 represents transition from starting station to first point after starting station
Matrix_Select=ones(Speed_N+1,N); % Data structure to record path selection for state transitions

%% Transition from second-to-last station to destination station
v_N=zeros(Speed_N+1,1);
v_N_1=zeros(Speed_N+1,1);
for i=1:1:Speed_N+1
    vi=step_v*(i-1);
    v_N(i,1)=vi;
    v_N_1(i,1)=0;
end
a_N=(v_N<=MaxCapacityV(N) & v_N_1<=MaxCapacityV(N+1)).*((v_N_1+v_N).*(v_N_1-v_N)/(2*step_s))+(v_N>MaxCapacityV(N) | v_N_1>MaxCapacityV(N+1)).*(1e16); % Speed constraints
R_N=GetBasicResistance(v_N*3.6)*M*g + wj(1,N)*1000; % Basic resistance + track additional resistance
Ft_max_N=GetTractionForce(v_N*3.6)*1000; % Maximum traction force, unit: N
Fd_max_N=GetMaxBrakeForce(v_N*3.6)*1000; % Maximum braking force, unit: N
F_N=(a_N>=a_min & a_N<=a_max).*(a_N*M*1000+R_N)+(a_N>a_max |a_N<a_min).*(1e16);
Cost_J_N=(F_N>Ft_max_N|F_N<-Fd_max_N).*(1e16)+(F_N>=0 & F_N<=Ft_max_N).*(F_N*step_s/Eta)+(F_N<0 & F_N>=-Fd_max_N).*(F_N*step_s*alpha_Re*Eta);
t_N=2*step_s./(v_N+v_N_1);
% Time constraints
Cost_N=lambda_T*abs(t_N-t_exp)+Cost_J_N;
Matrix_Jmin(:,N)=Cost_N;
Matrix_Select(:,N)=1;

%% Transition from starting station to second-to-last station
for j=N-1:-1:2 % From (N-1)th state to 2nd state transition, from starting station next point to second-to-last point
    
    v_j=zeros(Speed_N+1,Speed_N+1);
    v_j_1=zeros(Speed_N+1,Speed_N+1);
    for i=1:1:Speed_N+1
        vi=step_v*(i-1)*ones(1,Speed_N+1);
        v_j(i,:)=vi;
        v_j_1(i,:)=vi;
    end
    v_j_1=v_j_1'; % Transpose
    
    aj_j_1=(v_j<=MaxCapacityV(j) & v_j_1<=MaxCapacityV(j+1)).*((v_j_1+v_j).*(v_j_1-v_j)/(2*step_s))+(v_j>MaxCapacityV(j) | v_j_1>MaxCapacityV(j+1)).*(1e16); % Speed constraints
    % Acceleration matrix for transitions from all possible speeds at state j to all possible speeds at state j+1
    % a(m,n) represents required acceleration for transition from state j speed step_v*(m-1) to state j+1 speed step_v*(n-1)
    
    R=GetBasicResistance(v_j*3.6)*M*g + wj(1,j)*1000; % Basic resistance + track additional resistance
    Ft_max=GetTractionForce(v_j*3.6)*1000; % Maximum traction force, unit: N
    Fd_max=GetMaxBrakeForce(v_j*3.6)*1000; % Maximum braking force, unit: N
    
    F_k=(aj_j_1>=a_min & aj_j_1<=a_max).*(aj_j_1*M*1000+R)+(aj_j_1>a_max |aj_j_1<a_min).*(1e32);
    % Train force required for transitions from all possible speeds at state j to all possible speeds at state j+1
    
    Cost_J=(F_k>Ft_max|F_k<-Fd_max).*(1e32)+(F_k>=0 & F_k<=Ft_max).*(F_k*step_s/Eta)+(F_k<0 & F_k>=-Fd_max).*(F_k*step_s*alpha_Re*Eta);
    % Traction energy cost matrix
    
    t_k=2*step_s./(v_j+v_j_1);
    % Time constraints
    Cost=lambda_T*abs(t_k-t_exp)+Cost_J;
    CostT=Cost'+Matrix_Jmin(:,j+1);
    [value,index]=min(CostT);
    Matrix_Jmin(:,j)=value';
    Matrix_Select(:,j)=index';
    
end

%% Transition from starting station to first point after starting station
v_1=zeros(Speed_N+1,1);
v_2=zeros(Speed_N+1,1);
for i=1:1:Speed_N+1
    vi=step_v*(i-1);
    v_1(i,1)=0;
    v_2(i,1)=vi;
end
a_1=(v_1<=MaxCapacityV(1) & v_2<=MaxCapacityV(2)).*((v_2+v_1).*(v_2-v_1)/(2*step_s))+(v_1>MaxCapacityV(1) | v_2>MaxCapacityV(2)).*(1e16); % Speed constraints
R_1=GetBasicResistance(v_1*3.6)*M*g + wj(1,1)*1000; % Basic resistance + track additional resistance
Ft_max_1=GetTractionForce(v_1*3.6)*1000; % Maximum traction force, unit: N
Fd_max_1=GetMaxBrakeForce(v_1*3.6)*1000; % Maximum braking force, unit: N
F_1=(a_1>=a_min & a_1<=a_max).*(a_1*M*1000+R_1)+(a_1>a_max |a_1<a_min).*(1e16);
Cost_J_1=(F_1>Ft_max_1|F_1<-Fd_max_1).*(1e16)+(F_1>=0 & F_1<=Ft_max_1).*(F_1*step_s/Eta)+(F_1<0 & F_1>=-Fd_max_1).*(F_1*step_s*alpha_Re*Eta);
t_1=2*step_s./(v_1+v_2);
% Time constraints
Cost_1=lambda_T*abs(t_1-t_exp)+Cost_J_1;
Matrix_Jmin(:,1)=Cost_1;
Matrix_Select(:,1)=1:1:Speed_N+1;

%% Find the point with minimum first-step transition cost, then trace back optimal path through Matrix_Select

[value,flag]=min(Matrix_Jmin(:,1));

s=zeros(1,N+1); % Starting station as first point
v=zeros(1,N+1); % Starting station as first point
F=zeros(1,N+1); % Starting station as first point
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
x=['Time elapsed: ',num2str(t), ' s'];
disp(x)
end