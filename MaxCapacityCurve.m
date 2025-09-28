function [s,v]=MaxCapacityCurve(IsStop)
%% 输出
%s：距离离散点，1：N+1,以起始站为第一个点
%v：空间域离散的优化速度曲线，1：N+1，以起始站为第一个点
%% 输入
% IsStop：中间站是否停车的标志位，1表示停车
%% 
global step_s;
global startStation;
global endStation;
global Station %车站位置
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
s(1,1)=start_pos;
v(1,1)=v0;

for i=1:1:N
    if v(1,i)<SpdLimit(1,i)%牵引加速
        [curspeed] = CalculateOneStep('FP',v(1,i),1,i);
        if curspeed>SpdLimit(1,i)
            curspeed=SpdLimit(1,i);
        end
        v(1,i+1)=curspeed;
        s(1,i+1)=s(1,i)+step;
    end
    if v(1,i)==SpdLimit(1,i)%恒速
        v(1,i+1)=v(1,i);
        s(1,i+1)=s(1,i)+step;
    end
    if v(1,i)>SpdLimit(1,i)%制动减速
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
    if i==N %制动停车
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
% x=['SpeedLimit= ',num2str(SpdLimit(1,i)),' i= ',num2str(i),' v(1,i)= ',num2str(v(1,i))];
% disp(x);
end
end