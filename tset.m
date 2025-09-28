% Test script for train speed profile optimization
clc;

% Choose optimization mode for speed
fprintf('=== Dynamic Programming Speed Optimization ===\n');
fprintf('Choose mode:\n');
fprintf('1. Fast mode (recommended for testing)\n');
fprintf('2. Balanced mode (good accuracy + speed)\n');
fprintf('3. Accurate mode (high precision)\n');
fprintf('4. Original mode (very slow!)\n');

% Auto select balanced mode for demonstration
mode_choice = 2;
switch mode_choice
    case 1
        mode = 'fast';
    case 2
        mode = 'balanced';
    case 3
        mode = 'accurate';
    case 4
        mode = 'original';
end

fprintf('\nSelected: %s mode\n', mode);
fprintf('===============================================\n\n');

% Configure and load parameters
Global;  % Load basic parameters
ConfigureOptimization(mode);  % Apply optimization settings

% Reload dependent parameters
[wj,~,~]=GetAddResistance();
[Dis_Space,MaxCapacityV]=MaxCapacityCurve(1);

% Display problem size
fprintf('=== Problem Size Analysis ===\n');
fprintf('Distance: %.0f m\n', abs(end_pos - start_pos));
fprintf('Spatial steps (N): %d\n', N);
fprintf('Speed steps (Speed_N): %d\n', Speed_N);
fprintf('Matrix size per iteration: %d x %d\n', Speed_N+1, Speed_N+1);
fprintf('Total memory per iteration: %.1f MB\n', (Speed_N+1)^2 * 8 / 1024^2);
fprintf('Estimated speedup vs original: %.0fx\n', (1/0.01 / (1/step_v)) * (1/step_s));
fprintf('==============================\n\n');

global T;
global epsi_t;
[SpdLimit]=GetSpeedLimit(0);
[s,v,t]=MaxCapacityCurve(1);
s2=zeros(1,N);
s2(1,1:N)=s(1,2:N+1); 

% Plot speed limit and maximum capacity curves
figure(1)
plot(s2,SpdLimit,'--r','LineWidth',1.0)
hold on
plot(s,v,':g','LineWidth',1)
set(gca,'xdir','reverse')
hold on

% Iterative optimization to meet time constraints
tic;
lambda=632858.8509;
while(1)
    [s2,v2,F,T_real,E,Matrix_Jmin,Et,Eb,T1]=DynamicProgram(lambda);
    x=['lambda= ',num2str(lambda),' Actual running time= ',num2str(T_real),' Traction energy= ',num2str(E)];
    disp(x)
    if abs(T_real-T)<=epsi_t
        break;
    else
         lambda=lambda+((T_real-T)/T)*lambda;
    end
end
toc;

% Plot optimized speed profile
plot(s,v2*3.6,'k','LineWidth',1.5)
title('Single Section Energy-Optimal Speed Profile')
xlabel('Distance marker (m)')
ylabel('Speed (km/h)')
set(gca,'xdir','reverse')
hold off

% Plot force profile
figure(2)
plot(s2,F,'k','LineWidth',1.5)
title('Train Output Force Profile')
xlabel('Distance marker (m)')
ylabel('Train output force (N)')
set(gca,'xdir','reverse')

% Plot traction and braking characteristics
v=0:0.0001:80;
B=GetMaxBrakeForce(v);
F=GetTractionForce(v);

subplot(1,2,1)
plot(v,F,'k','LineWidth',2.0)
title('Train Traction Characteristics')
xlabel('Speed (km/h)')
ylabel('Force (kN)')
axis([0 80 80 220])

subplot(1,2,2)
plot(v,B,'k','LineWidth',2.0)
title('Train Braking Characteristics')
xlabel('Speed (km/h)')
ylabel('Force (kN)')
axis([0 80 152 168])