function [ Force ] = GetTractionForce( v )
% Get train traction force (maximum traction force under current speed)
% Input parameters:
    % v: Train speed, unit: km/h
% Output parameters:
    % Force: Train traction force, unit: KN    
% HXD2 traction force
Force=(v>=0 & v<=51.5).*(203)+(51.5<v & v<=80).*(-0.002032.*v.^3+0.4928.*v.^2-42.13.*v+1343);

end