function [ output_args ] = GetBasicResistance( speed )
% Calculate train basic running resistance
% Input parameters:
    % speed: Train running speed, km/h
% Output parameters:
    % Resistance, unit: N/KN (Newton per kilonewton)
% a=2.031;
% b=0.0622;
% c=0.001807;
% output_args=a+b.*speed+c.*speed.*speed;

output_args=(0.92+0.0048.*speed+0.000125.*speed.^2);
end