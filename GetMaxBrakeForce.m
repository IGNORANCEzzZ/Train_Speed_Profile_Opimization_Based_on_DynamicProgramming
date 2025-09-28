function [ Force ] = GetMaxBrakeForce( v )
% Calculate maximum train electric braking force (envelope)
% Input parameters:
    % v: Train speed, unit: km/h
% Output parameters:
    % Force: Train regenerative braking force, unit: KN
Force=166*(0<=v&v<=77)+(0.1343*(v.^2)-25.07*v+1300).*(77<v&v<=80);
end