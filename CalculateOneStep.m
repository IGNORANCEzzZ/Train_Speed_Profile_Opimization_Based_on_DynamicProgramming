function [curspeed] = CalculateOneStep(mode,preSpeed,Forward,k)
% Calculate train speed at x+dx or x-dx based on previous speed, controlled by Forward parameter
% Input parameters:
     % mode: Current operating mode
     % preSpeed: Speed at current position, km/h
     % Forward: Whether to calculate speed at x+dx or x-dx
     % k: Position index in discrete spatial domain
% Output parameters:
     % curspeed: Speed at x+dx or x-dx, km/h
%%
global M;       % Total train weight, t
global g;       % Gravitational acceleration
global step_s;  % Distance step size, m
global wj;
%%
x_step=step_s;
%%
switch (mode)  
    case 'FP'
        TractionForce=GetTractionForce(preSpeed)*1000; % Original unit is KN, multiply by 1000 to get N
        % Maximum traction force based on speed
        BreakForce=0; 
        % Maximum electric braking force

    case 'C'
        TractionForce=0;
        BreakForce=0;
    case 'FB'
        TractionForce=0;
        BreakForce=GetMaxBrakeForce(preSpeed)*1000; % Original unit is KN, multiply by 1000 to get N
        % Function to get maximum electric braking force based on speed
end
BasicForce=GetBasicResistance(preSpeed)*M*g; % N
% Train basic running resistance based on speed

AdditionalForce=wj(k)*1000; % N
% Additional gradient resistance from train position (gradient), not considering curves and tunnels
if k<10
disp('BasicForce= ')
disp(BasicForce)
disp('AdditionalForce= ')
disp(AdditionalForce)
end
a=(TractionForce-BreakForce-BasicForce-AdditionalForce)/(M*1000);

%%
% a is acceleration; a(m/s^2)=F/m(N/kg), M unit is tons, need to multiply by 1000 to get kg
preSpeed=preSpeed/3.6; % Convert from km/h to m/s
    if Forward
        % If Forward=1, curspeed is speed at (prePos+dx)
        curspeed=sqrt(preSpeed*preSpeed+2*x_step*a)*3.6; % Convert from m/s to km/h
        % Formula: V^2-V0^2=2aL, so V^2=V0^2+2*dx*a

    else
        % If Forward=0, curspeed is speed at (prePos-dx)
        curspeed=sqrt(preSpeed*preSpeed-2*x_step*a)*3.6; % Convert from m/s to km/h

    end

end