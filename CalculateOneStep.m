function [curspeed] = CalculateOneStep(mode,preSpeed,Forward,k)
%��������������Ǽ���x+dx��x-dx�����ٶȣ�ȡ����Forward��ֵ
%���������
     %mode����ǰ����
     %preSpeed����ǰλ�õ��ٶ�,km/h
     %Forward�������Ǽ���x+dx���ٶȻ��Ǽ���x-dx���ٶ�
     %k,��ɢ�ռ���ĵڼ���λ��
%���������
     %curspeed��x+dx��x-dx�����ٶ�km/h
%%
global M;       %�г����� t
global g;       %�������ٶ�
global step_s;   %���벽�� m
global wj;
%%
x_step=step_s;
%%
switch (mode)  
    case 'FP'
        TractionForce=GetTractionForce(preSpeed)*1000;%ԭ�����ĵ�λ��KN�����������1000���N
        %���ٶȵõ������ǣ����
        BreakForce=0; 
        %�����ƶ���

    case 'C'
        TractionForce=0;
        BreakForce=0;
    case 'FB'
        TractionForce=0;
        BreakForce=GetMaxBrakeForce(preSpeed)*1000;%ԭ�����ĵ�λ��KN�����������1000���N
        %���ٶȵõ������ƶ����ĺ���
end
BasicForce=GetBasicResistance(preSpeed)*M*g;%N
%���ٶȵõ����г�������������

AdditionalForce=wj(k)*1000;%N
%���г�����λ�ã��µ����õ����µ��������������������ߺ��������������������µ�����������
a=(TractionForce-BreakForce-BasicForce-AdditionalForce)/(M*1000);


%%
%a�Ǽ��ٶȣ�a(m/s^2)=F/m(N/kg),M�ĵ�λ�Ƕ֣�������Ҫ����1000���Kg
preSpeed=preSpeed/3.6;%��ǧ��ÿСʱת����m/s
%����3.6��ʲô��˼���
    if Forward
        %���Forward=1����curspeed���ǣ�prePos+dx�������ٶ�
        curspeed=sqrt(preSpeed*preSpeed+2*x_step*a)*3.6;%��m/sת����km/h
        %��ʽ��V^2-VO^2=2aL��V^2=V0^2+2*dx*a

    else
        %���Forward=0����curspeed���ǣ�prePos-dx�������ٶ�
        curspeed=sqrt(preSpeed*preSpeed-2*x_step*a)*3.6;%��m/sת����km/h

    end

end