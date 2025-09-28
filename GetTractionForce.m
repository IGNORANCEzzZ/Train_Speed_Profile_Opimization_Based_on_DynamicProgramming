function [ Force ] = GetTractionForce( v )
% Calculate maximum train traction force (traction characteristic curve envelope)
% Input parameters:
    % v: Train speed, unit: km/h
% Output parameters:
    % Force: Train traction force, unit: KN    
% HXD2 traction characteristics
Force=203*(0<=v&v<=51.5)+(-0.002032.*(v.^3)+0.4928.*(v.^2)-42.13.*v+1343).*(51.5<v&v<=80);

end