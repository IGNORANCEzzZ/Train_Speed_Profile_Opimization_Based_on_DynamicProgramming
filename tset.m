
% v=0:0.001:100;
% [ Force ] = GetTractionForce( v );
% plot(v,Force)
% [ Force ] = GetMaxBrakeForce( v );
% figure(2)
% plot(v,Force)

clc;
clear all
Global;
[SpdLimit]=GetSpeedLimit(1);
[s,v]=MaxCapacityCurve(1);
s2=zeros(1,N);
s2(1,1:N)=s(1,2:N+1);

plot(s2,SpdLimit,'r','LineWidth',1.5)
hold on
plot(s,v,'g','LineWidth',1.5)
set(gca,'xdir','reverse')
hold on
[s2,v2,F,T,E,Matrix_Jmin]=DynamicProgram();
plot(s2,v2*3.6,'b','LineWidth',1.5)
set(gca,'xdir','reverse')