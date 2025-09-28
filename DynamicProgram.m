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

%% Pre-compute speed matrices to avoid repeated calculations
% Create speed vectors for all possible transitions
speed_indices = (0:Speed_N)*step_v;  % All possible speeds
speed_kmh = speed_indices * 3.6;     % Convert to km/h for resistance calculations

% Pre-compute force characteristics for all speeds (vectorized)
Ft_max_all = GetTractionForce(speed_kmh) * 1000;  % All traction forces
Fd_max_all = GetMaxBrakeForce(speed_kmh) * 1000;  % All braking forces
R_basic_all = GetBasicResistance(speed_kmh) * M * g;  % All basic resistances

% Pre-compute speed transition matrices (used in all j loops)
v_grid = repmat(speed_indices', 1, Speed_N+1);  % Current speeds
v_next_grid = repmat(speed_indices, Speed_N+1, 1);  % Next speeds

% Pre-compute acceleration matrix
a_grid = (v_next_grid.^2 - v_grid.^2) / (2*step_s);

% Pre-compute time matrix (avoid division by zero)
time_denominator = v_grid + v_next_grid;
time_denominator(time_denominator == 0) = eps;  % Avoid division by zero
t_grid = 2*step_s ./ time_denominator;

% Constants for performance
M_1000 = M * 1000;
step_s_over_Eta = step_s / Eta;
alpha_Eta_step_s = alpha_Re * Eta * step_s;

Matrix_Jmin=inf*ones(Speed_N+1,N); % Data structure to record minimum cumulative cost of state transitions
% Row i represents speed (i-1)*step_v, column j represents transition from spatial point j to destination (N+1 point), recording minimum cumulative cost Jmin
% Element (i,j) represents minimum cumulative cost Jmin from spatial point j with speed (i-1)*step_v to destination (N+1 point)
% j=1:N, j=1 represents transition from starting station to first point after starting station
Matrix_Select=ones(Speed_N+1,N); % Data structure to record path selection for state transitions

%% Transition from second-to-last station to destination station
% Use pre-computed values for terminal state
speed_constraint_N = (speed_indices <= MaxCapacityV(N)) & (0 <= MaxCapacityV(N+1));
a_N = speed_constraint_N .* (-speed_indices.^2 / (2*step_s)) + (~speed_constraint_N) * 1e16;

R_N = R_basic_all(1:Speed_N+1) + wj(1,N)*1000;
F_N = (a_N >= a_min & a_N <= a_max) .* (a_N * M_1000 + R_N) + ...
      (a_N > a_max | a_N < a_min) * 1e16;

% Vectorized cost calculation
force_feasible = (F_N <= Ft_max_all(1:Speed_N+1)) & (F_N >= -Fd_max_all(1:Speed_N+1));
Cost_J_N = (~force_feasible) * 1e16 + ...
           force_feasible .* ((F_N >= 0) .* (F_N * step_s_over_Eta) + ...
                              (F_N < 0) .* (F_N * alpha_Eta_step_s));

t_N = 2*step_s ./ (speed_indices + eps);  % Avoid division by zero
Cost_N = lambda_T * abs(t_N - t_exp) + Cost_J_N;

Matrix_Jmin(:,N) = Cost_N;
Matrix_Select(:,N) = 1;

%% Transition from starting station to second-to-last station
for j=N-1:-1:2 % From (N-1)th state to 2nd state transition
    
    % Speed constraints for current and next positions
    speed_valid_j = speed_indices <= MaxCapacityV(j);
    speed_valid_j1 = speed_indices <= MaxCapacityV(j+1);
    
    % Create constraint mask
    constraint_mask = speed_valid_j' & speed_valid_j;
    
    % Calculate acceleration with constraints
    aj_j_1 = constraint_mask .* a_grid + (~constraint_mask) * 1e16;
    
    % Track resistance for current position
    R_current = R_basic_all + wj(1,j)*1000;
    R_matrix = repmat(R_current', 1, Speed_N+1);
    
    % Force calculation
    accel_valid = (aj_j_1 >= a_min) & (aj_j_1 <= a_max);
    F_k = accel_valid .* (aj_j_1 * M_1000 + R_matrix) + (~accel_valid) * 1e32;
    
    % Force constraint checking
    Ft_matrix = repmat(Ft_max_all', 1, Speed_N+1);
    Fd_matrix = repmat(Fd_max_all', 1, Speed_N+1);
    force_feasible = (F_k <= Ft_matrix) & (F_k >= -Fd_matrix);
    
    % Energy cost calculation
    Cost_J = (~force_feasible) * 1e32 + ...
             force_feasible .* ((F_k >= 0) .* (F_k * step_s_over_Eta) + ...
                                (F_k < 0) .* (F_k * alpha_Eta_step_s));
    
    % Time cost
    Cost = lambda_T * abs(t_grid - t_exp) + Cost_J;
    
    % Add future costs and find minimum
    CostT = Cost' + Matrix_Jmin(:,j+1);
    [value, index] = min(CostT, [], 1);
    Matrix_Jmin(:,j) = value';
    Matrix_Select(:,j) = index';
    
end

%% Transition from starting station to first point after starting station
v_1=zeros(Speed_N+1,1);
v_2=zeros(Speed_N+1,1);
for i=1:1:Speed_N+1
    vi=step_v*(i-1);
    v_1(i,1)=0;
    v_2(i,1)=vi;
end
a_1=(v_1<=MaxCapacityV(1) & v_2<=MaxCapacityV(2)).*((v_2+v_1).*(v_2-v_1)/(2*step_s))+(v_1>MaxCapacityV(1) | v_2>MaxCapacityV(2)).*(1e16); % Acceleration constraints
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

% Pre-allocate result arrays
s=zeros(1,N+1); % Starting station as first point
v=zeros(1,N+1); % Starting station as first point
F=zeros(1,N+1); % Starting station as first point
s(1,1)=start_pos;
v(1,1)=v0;
F(1,N+1)=0;

% Vectorized result reconstruction
T=0; E=0; Et=0; Eb=0; T1=0;
for i=1:N
    s(1,i+1)=s(1,i)+step;
    flag=Matrix_Select(flag,i);
    v(1,i+1)=step_v*(flag-1);
    
    % Vectorized calculations
    v_avg = (v(1,i)+v(1,i+1))/2;
    dt = 2*step_s/(v(1,i)+v(1,i+1));
    
    if s(1,i+1)>=12240
       T1=T1+dt;
    end
    
    T=T+dt;
    a=(v(1,i+1)^2-v(1,i)^2)/(2*step_s);
    F(1,i)=a*M_1000+GetBasicResistance(v(1,i)*3.6)*M*g + wj(1,i)*1000;
    
    % Energy calculation
    if F(1,i)>=0
        dE = F(1,i)*step_s_over_Eta;
        Et = Et + dE;
    else
        dE = F(1,i)*alpha_Eta_step_s;
        Eb = Eb + dE;
    end
    E = E + dE;
end

t=toc;
fprintf('Time elapsed: %.3f s\n', t);
end