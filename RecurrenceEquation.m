function [J_k,t_k,F_k,e_k]=RecurrenceEquation(v_k,v_k_1,k)
%% ���룺
%v_k��ǰһ����ɢ����ٶȣ���λm/s
%v_k_1:��һ����ɢ����ٶȣ���λm/s
%k���ռ����k���㣬���𳵵�Ϊ��һ����
%% �����
% J_k����k��ת�Ƶ�k+1������Ҫ�Ĵ���
% t_k����k��ת�Ƶ�k+1������Ҫ������ʱ�䣬��λs
% F_k����k��ת�Ƶ�k+1����ʹ�õ�����ǣ�����ƶ���������λN
% e_k����k��ת�Ƶ�k+1����ʹ�õĿ��������ƶ����ܺģ���λ������J
%%
global step_s;
global M;
global g;
global wj;
global t_exp;
global lambda_T;
global Eta;
global alpha_Re;
global MaxCapacityV;%1:N+1,��������ٶ�����
global a_max;
global a_min;

a=(v_k_1^2-v_k^2)/(2*step_s);%��λm/s^2
w0=GetBasicResistance(v_k*3.6)*M*g;%��λN/KN * KKG/ N/KG=N
w_j=wj(1,k)*1000;%��λKN*1000=N��
F_k=a*M*1000+w0+w_j;
if v_k+v_k_1==0
    t_k=inf;
else
    t_k=2*step_s/(v_k+v_k_1);%ʹ��ƽ���ٶ���t_k
end
Ft_max=GetTractionForce(v_k*3.6)*1000;%���ٶ��µ����ǣ��������λN
Fd_max=GetMaxBrakeForce(v_k*3.6)*1000;%���ٶ��µ�����ƶ�������λN
x=['a= ',num2str(a),' F_k= ',num2str(F_k),' w0= ',num2str(w0),' w_j= ',num2str(w_j),' t_k= ',num2str(t_k),' MaxCapacityV(1,k)=',num2str(MaxCapacityV(1,k)),' MaxCapacityV(1,k+1)=',num2str(MaxCapacityV(1,k+1))];
global N;
if k==N
disp(x)
end
%% �ж�����������ٶ�֮���״̬ת���Ƿ���� ���ٶ�Լ�������ǣ�����ƶ���Լ�����Լ������������Լ��
if a>=a_min && a<= a_max && F_k>=-Fd_max && F_k<=Ft_max && v_k*3.6<=MaxCapacityV(1,k) && v_k_1*3.6<=MaxCapacityV(1,k+1)
% if a>=a_min && a<= a_max && F_k>=-Fd_max && F_k<=Ft_max
    if F_k>=0
        J_k=F_k/Eta*step_s+lambda_T*abs(t_k-t_exp);
    else
        J_k=-alpha_Re*Eta*F_k*step_s+lambda_T*abs(t_k-t_exp);
    end
else
    J_k=inf;
end

if F_k>=0
    e_k=F_k/Eta*step_s;
else
    e_k=-alpha_Re*Eta*F_k*step_s;
end