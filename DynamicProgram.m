function [s,v,F,T,E,Matrix_Jmin]=DynamicProgram()
%% Outputs:
% s: Discrete spatial points, unit: m
% v: Optimized speed profile, unit: km/h
% F: Train forces at each spatial point, unit: N
% T: Train running time, unit: s
% E: Total energy consumption (traction energy), unit: J

%% Dynamic programming solution for energy-optimal train control
global Speed_N;
global N;
global step_s;
global step_v;
global v0;
global vend;
global start_pos;
global end_pos;
global MaxCapacityV; % Maximum capacity speeds, 1:N+1
if end_pos>start_pos
    step=step_s;
else
   step=-step_s;
end

Matrix_Jmin=inf*ones(Speed_N+1,N); % Data structure to record minimum cumulative cost of state transitions

% Row i represents speed (i-1)*step_v, column j represents transition from spatial point j to destination (N+1 point), recording minimum cumulative cost Jmin
% Element (i,j) represents minimum cumulative cost Jmin from spatial point j with speed (i-1)*step_v to destination (N+1 point)
% j=1:N, j=1 represents transition from starting station to first point after starting station

Matrix_T=zeros(Speed_N+1,N); % Data structure to record cumulative time of state transitions
Matrix_F=zeros(Speed_N+1,N); % Data structure to record train forces of state transitions
Matrix_E=zeros(Speed_N+1,N); % Data structure to record cumulative energy consumption of state transitions
Matrix_Select=ones(Speed_N+1,N); % Data structure to record path selection for state transitions

%% Transition from second-to-last station to destination station
[L,U]=RangeDecrease(N,0,0);
for i=L:1:U % No need to select the best path at last column because destination speed is 0, destination has only one speed state
    [J_k,t_k,F_k,e_k]=RecurrenceEquation(step_v*(i-1),0,N);
    Matrix_Jmin(i,N)=J_k;
    Matrix_T(i,N)=t_k;
    Matrix_F(i,N)=F_k;
    Matrix_E(i,N)=e_k;
    Matrix_Select(i,N)=1; % Always select speed 0
end

%% Transition from starting station to second-to-last station
for j=N-1:-1:2 % From (N-1)th state to 2nd state transition, from starting station next point to second-to-last point
    L_pre=L;
    U_pre=U;
    disp(L)
    disp(U)
    [L,U]=RangeDecrease(j,L,U); % Possible range for transition from j+1 state to j state
    for i=1:1:U
        x=['j= ',num2str(j),' i= ',num2str(i)];
        disp(x);
        if MaxCapacityV(j+1)==0 % Possible range for transition from j state with speed i to j+1 state
            L3=1;
            U3=1;
        else
            [L2,U2]=RangeDecrease(j+1,i,i);
            L3=min(L_pre,L2);
            U3=max(U_pre,U2);
        end
        z=L3;
        [J_k,t_k,F_k,e_k]=RecurrenceEquation(step_v*(i-1),step_v*(z-1),j);
        J_selected=J_k+Matrix_Jmin(z,j+1);
        t_selected=t_k+Matrix_T(z,j+1);
        F_selected=F_k;
        E_selected=e_k+Matrix_E(z,j+1);
        select=z;
        for z=L3:1:U3 % Select optimal transition path at point j (j=N-1:1:2), speed step_v*(i-1) to destination
            [J_k,t_k,F_k,e_k]=RecurrenceEquation(step_v*(i-1),step_v*(z-1),j);
            if J_k+Matrix_Jmin(z,j+1)<J_selected
                J_selected=J_k+Matrix_Jmin(z,j+1);
                t_selected=t_k+Matrix_T(z,j+1);
                F_selected=F_k;
                E_selected=e_k+Matrix_E(z,j+1);
                select=z;
            end
        end
        Matrix_Jmin(i,j)=J_selected;
        Matrix_T(i,j)=t_selected;
        Matrix_F(i,j)=F_selected;
        Matrix_E(i,j)=E_selected;
        Matrix_Select(i,j)=select; % select means transition to speed step_v*(select-1)
    end
end

%% Transition from starting station to first point after starting station
for i=1:1:Speed_N+1
    [J_k,t_k,F_k,e_k]=RecurrenceEquation(0,step_v*(i-1),1);
    Matrix_Jmin(i,1)=J_k+Matrix_Jmin(i,2);
    Matrix_T(i,1)=t_k+Matrix_T(i,2);
    Matrix_F(i,1)=F_k;
    Matrix_E(i,1)=e_k+Matrix_E(i,2);
    Matrix_Select(i,1)=i;
end

%% Find the point with minimum first-step transition cost, then trace back optimal path through Matrix_Select

J_min=Matrix_Jmin(1,1);
flag=1;
for i=2:1:Speed_N+1
    if Matrix_Jmin(i,1)<J_min
        J_min=Matrix_Jmin(i,1);
        flag=i;
    end
end

T=Matrix_T(flag,1);
E=Matrix_E(flag,1);
s=zeros(1,N+1); % Starting station as first point
v=zeros(1,N+1); % Starting station as first point
F=zeros(1,N+1); % Starting station as first point
s(1,1)=start_pos;
v(1,1)=v0;
F(1,N+1)=0;
disp(Matrix_Jmin(:,N));

for i=1:1:N
    s(1,i+1)=s(1,i)+step;
    flag=Matrix_Select(flag,i);
    v(1,i+1)=step_v*(flag-1);
    F(1,i)=Matrix_F(flag,i);
end
end