function [ output_args ] = GetBasicResistance( speed )
% Calculate train basic running resistance
% Input parameters:
    % speed: Train running speed, km/h
% Output parameters:
    % Basic resistance, unit: N/KN (Newton per kilonewton)
a=2.031;
b=0.0622;
c=0.001807;
output_args=a+b*speed+c*speed*speed;
end