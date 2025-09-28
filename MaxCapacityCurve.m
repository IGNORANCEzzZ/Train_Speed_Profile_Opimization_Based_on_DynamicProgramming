function [s,v]=MaxCapacityCurve(IsStop)
%% ���
%s��������ɢ�㣬1��N+1,����ʼվΪ��һ����
%v���ռ�����ɢ���Ż��ٶ����ߣ�1��N+1������ʼվΪ��һ����
%% ����
% IsStop���м�վ�Ƿ�ͣ���ı�־λ��1��ʾͣ��
%% 
global step_s;
global startStation;
global endStation;
global Station %��վλ��
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
    if v(1,i)<SpdLimit(1,i)%ǣ������
        [curspeed] = CalculateOneStep('FP',v(1,i),1,i);
        if curspeed>SpdLimit(1,i)
            curspeed=SpdLimit(1,i);
        end
        v(1,i+1)=curspeed;
        s(1,i+1)=s(1,i)+step;
    end
    if v(1,i)==SpdLimit(1,i)%����
        v(1,i+1)=v(1,i);
        s(1,i+1)=s(1,i)+step;
    end
    if v(1,i)>SpdLimit(1,i)%�ƶ�����
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
    if i==N %�ƶ�ͣ��
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