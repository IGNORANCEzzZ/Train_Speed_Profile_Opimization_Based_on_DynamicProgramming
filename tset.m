clc;
Global;

global T;
global epsi_t;
[SpdLimit]=GetSpeedLimit(0);
[s,v,t]=MaxCapacityCurve(1);
s2=zeros(1,N);
s2(1,1:N)=s(1,2:N+1); 

figure(1)
plot(s2,SpdLimit,'--r','LineWidth',1.0)
hold on
plot(s,v,':g','LineWidth',1)
set(gca,'xdir','reverse')
hold on

tic;
lambda=632858.8509;
while(1)
    [s2,v2,F,T_real,E,Matrix_Jmin,Et,Eb,T1]=DynamicProgram(lambda);
    x=['lambda= ',num2str(lambda),' 实际运行时间= ',num2str(T_real),' 牵引能耗= ',num2str(E)];
    disp(x)
    if abs(T_real-T)<=epsi_t
        break;
    else
         lambda=lambda+((T_real-T)/T)*lambda;
%         if T_real>T
%             lambda=1.5*lambda;
%         elseif T_real<T
%             lambda=0.5*lambda;
%         end
    end
end
toc;

plot(s,v2*3.6,'k','LineWidth',1.5)
title('单区间节能优化速度曲线')
xlabel('公里标(m)')
ylabel('速度(km/h)')
set(gca,'xdir','reverse')
hold off

figure(2)
plot(s2,F,'k','LineWidth',1.5)
title('列车输出力结果图')
xlabel('公里标(m)')
ylabel('列车输出力(N)')
set(gca,'xdir','reverse')

v=0:0.0001:80;
B=GetMaxBrakeForce(v);
F=GetTractionForce(v);

subplot(1,2,1)
plot(v,F,'k','LineWidth',2.0)
title('列车牵引特性曲线')
xlabel('速度/(km/h)')
ylabel('力/kN')
axis([0 80 80 220])

subplot(1,2,2)
plot(v,B,'k','LineWidth',2.0)
title('列车制动特性曲线')
xlabel('速度/(km/h)')
ylabel('力/kN')
axis([0 80 152 168])