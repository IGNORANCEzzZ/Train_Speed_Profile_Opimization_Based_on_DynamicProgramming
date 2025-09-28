function [ Force ] = GetMaxBrakeForce( v )
%计算列车最大电制动力（外包络）
%输入参数：
    %v 列车速度，单位：km/h
%输出参数：
    %Force 列车再生制动力 单位：KN
Force=166*(0<=v&v<=77)+(0.1343*(v.^2)-25.07*v+1300).*(77<v&v<=80);
end