function [ Force ] = GetMaxBrakeForce( v )
% Get train maximum braking force (maximum braking force under current speed)
% Input parameters:
    % v: Train speed, unit: km/h
% Output parameters:
    % Force: Train braking force, unit: KN
Force=(v>=0 & v<=77).*(166)+(77<v & v<=80).*(0.1343.*v.^2-25.07.*v+1300);
end