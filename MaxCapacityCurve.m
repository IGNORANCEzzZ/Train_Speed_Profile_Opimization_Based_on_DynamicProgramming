function [s,v,t]=MaxCapacityCurve(IsStop)
%% Outputs:
% s: Spatial discrete points, 1:N+1, starting station as first point
% v: Speed profile for spatial discrete points, 1:N+1, starting station as first point
%% Input:
% IsStop: Flag indicating whether to stop at intermediate stations, 1 means stop
%% 
global step_s;
global startStation;
global endStation;
global Station % Station positions
global start_pos;
global end_pos;
global N;
global v0;
global vend;
if end_pos>start_pos
    step=step_s;
else
   step=-step_s;
end

[SpdLimit]=GetSpeedLimit(IsStop);
s=zeros(1,N+1);
v=zeros(1,N+1);
t=zeros(1,N+1);
s(1,1)=start_pos;
v(1,1)=v0;
t(1,1)=0;

for i=1:1:N
    if v(1,i)<SpdLimit(1,i) % Traction phase
        [curspeed] = CalculateOneStep('FP',v(1,i),1,i);
        if curspeed>SpdLimit(1,i)
            curspeed=SpdLimit(1,i);
        end
        v(1,i+1)=curspeed;
        s(1,i+1)=s(1,i)+step;
    end
    if v(1,i)==SpdLimit(1,i) % Coasting phase
        v(1,i+1)=v(1,i);
        s(1,i+1)=s(1,i)+step;
    end
    if v(1,i)>SpdLimit(1,i) % Braking phase
        v(1,i+1)=SpdLimit(1,i);
        s(1,i+1)=s(1,i)+step;
        j=i;
        v_j=CalculateOneStep('FB',v(1,j+1),0,j);
        while v_j<v(1,j)
            v(1,j)=v_j;
            v_j=CalculateOneStep('FB',v(1,j),0,j-1);
            j=j-1;
        end
    end
    if i==N % Braking to stop
        v(1,i+1)=vend;
        s(1,i+1)=s(1,i)+step;
        j=i;
        v_j=CalculateOneStep('FB',v(1,j+1),0,j);
        while v_j<v(1,j)
            v(1,j)=v_j;
            v_j=CalculateOneStep('FB',v(1,j),0,j-1);
            j=j-1;
        end
    end
end
for i=1:1:N
    t(1,i+1)=t(1,i)+2*step_s/((v(1,i)+v(1,i+1))/3.6);
end
end