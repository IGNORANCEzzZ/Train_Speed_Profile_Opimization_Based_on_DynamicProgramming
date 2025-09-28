function [curspeed] = CalculateOneStep(mode,preSpeed,Forward,k)
%这个函数的作用是计算x+dx或x-dx处的速度，取决于Forward的值
%输入参数：
     %mode：当前工况
     %preSpeed：当前位置的速度,km/h
     %Forward：决定是计算x+dx的速度还是计算x-dx的速度
     %k,离散空间域的第几个位置
%输出参数：
     %curspeed：x+dx或x-dx处的速度km/h
%%
global M;       %列车总重 t
global g;       %重力加速度
global step_s;   %距离步长 m
global wj;
%%
x_step=step_s;
%%
switch (mode)  
    case 'FP'
        TractionForce=GetTractionForce(preSpeed)*1000;%原函数的单位是KN，所以这里乘1000变成N
        %由速度得到的最大牵引力
        BreakForce=0; 
        %最大电制动力

    case 'C'
        TractionForce=0;
        BreakForce=0;
    case 'FB'
        TractionForce=0;
        BreakForce=GetMaxBrakeForce(preSpeed)*1000;%原函数的单位是KN，所以这里乘1000变成N
        %由速度得到最大电制动力的函数
end
BasicForce=GetBasicResistance(preSpeed)*M*g;%N
%由速度得到的列车基本运行阻力

AdditionalForce=wj(k)*1000;%N
%由列车所处位置（坡道）得到的坡道附加阻力（不考虑曲线和隧道，这个阻力即加算坡道附加阻力）
if k<10
disp('BasicForce= ')
disp(BasicForce)
disp('AdditionalForce= ')
disp(AdditionalForce)
end
a=(TractionForce-BreakForce-BasicForce-AdditionalForce)/(M*1000);


%%
%a是加速度；a(m/s^2)=F/m(N/kg),M的单位是吨，所以需要乘上1000变成Kg
preSpeed=preSpeed/3.6;%从千米每小时转化成m/s
%除以3.6是什么意思变成
    if Forward
        %如果Forward=1，则curspeed就是（prePos+dx）处的速度
        curspeed=sqrt(preSpeed*preSpeed+2*x_step*a)*3.6;%从m/s转化成km/h
        %公式：V^2-VO^2=2aL即V^2=V0^2+2*dx*a

    else
        %如果Forward=0，则curspeed就是（prePos-dx）处的速度
        curspeed=sqrt(preSpeed*preSpeed-2*x_step*a)*3.6;%从m/s转化成km/h

    end

end