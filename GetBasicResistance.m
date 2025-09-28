function [ output_args ] = GetBasicResistance( speed )
%计算列车运行基本阻力
%输入参数：
    %speed：列车运行速度,km/h
%输出参数：
    %阻力 单位：N/KN(牛每千牛)
% a=2.031;
% b=0.0622;
% c=0.001807;
% output_args=a+b.*speed+c.*speed.*speed;

output_args=(0.92+0.0048.*speed+0.000125.*speed.^2);
end
