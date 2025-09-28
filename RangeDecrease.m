function [L,U]=RangeDecrease(j,L_pre,U_pre)
global step_v;
global step_s; % Step size
global a_max; % Maximum acceleration, m/s^2
global a_min; % Minimum acceleration, m/s^2
global MaxCapacityV;
global Speed_N;

maxV_pre=step_v*(U_pre-1);
maxV_after=sqrt(2*a_max*step_s+maxV_pre^2);
maxV=min(MaxCapacityV(j),maxV_after);
U=ceil(maxV/step_v+1); % Round up

minV_pre=step_v*(L_pre-1);
z=2*a_min*step_s+minV_pre^2;
if z<0
    z=0;
end
minV_after=sqrt(z);
minV=max(0,minV_after);
L=fix(minV/step_v+1); % Round down
L=L+1;
U=U+1;
if L<=1
    L=1;
end
if U>=Speed_N
    U=Speed_N;
end
end