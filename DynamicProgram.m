function [s,v,F,T,E,Matrix_Jmin,Et,Eb,T1]=DynamicProgram(lambda)

%% ���
% s: ��ɢ�Ŀռ��ᣬ��λm
% v���Ż�����ٶ����ߣ���λkm/h
% F: �г�������ڿռ����ϵ���ɢ���У���λN
% T���г�������ʱ�䣬��λs
% E�����������ƶ����г���ǣ���ܺ�,��λJ

%% �Ե׶��ϵĳ��涯̬�滮��ⷽ��
global Speed_N;
global N;
global step_s;
global step_v;
global v0;
global start_pos;
global end_pos;
global MaxCapacityV;%1:N+1,��������ٶ�����

global wj;
global t_exp;
global Eta;
global alpha_Re;
global a_max;
global a_min;
global M;
global g;

lambda_T=lambda;

if end_pos>start_pos
    step=step_s;
else
   step=-step_s;
end

Matrix_Jmin=inf*ones(Speed_N+1,N);%���ݼ�¼���� ֮ ״̬ת�ƴ����ۼƾ���
%��i�б�ʾ(i-1)��v���ٶȣ���j�б�ʾ�ռ����ϵ�j����ת�Ƶ��յ㣨��N+1���㣩����Ҫ����С���ܴ���Jmin
%��i,j����ʾ�ӿռ����ϵ�j���㣬���ٶ�(i-1)��vת�Ƶ��յ㣨��N+1���㣩����Ҫ����С���ܴ���Jmin
%j=1:N��j=1��ʾ����ʼվת�Ƶ���ʼվ��һ����
Matrix_Select=ones(Speed_N+1,N);%���ݼ�¼���� ֮ ״̬ת������·��ѡ�����

%% �յ�վǰһ����ת�Ƶ��յ�վ
v_N=zeros(Speed_N+1,1);
v_N_1=zeros(Speed_N+1,1);
for i=1:1:Speed_N+1
    vi=step_v*(i-1);
    v_N(i,1)=vi;
    v_N_1(i,1)=0;
end
a_N=(v_N<=MaxCapacityV(N) & v_N_1<=MaxCapacityV(N+1)).*((v_N_1+v_N).*(v_N_1-v_N)/(2*step_s))+(v_N>MaxCapacityV(N) | v_N_1>MaxCapacityV(N+1)).*(1e16);%���ٶȾ���
R_N=GetBasicResistance(v_N*3.6)*M*g + wj(1,N)*1000;%����������������·������������
Ft_max_N=GetTractionForce(v_N*3.6)*1000;%���ǣ�������󣬵�λN
Fd_max_N=GetMaxBrakeForce(v_N*3.6)*1000;%����ƶ������󣬵�λN
F_N=(a_N>=a_min & a_N<=a_max).*(a_N*M*1000+R_N)+(a_N>a_max |a_N<a_min).*(1e16);
Cost_J_N=(F_N>Ft_max_N|F_N<-Fd_max_N).*(1e16)+(F_N>=0 & F_N<=Ft_max_N).*(F_N*step_s/Eta)+(F_N<0 & F_N>=-Fd_max_N).*(F_N*step_s*alpha_Re*Eta);
t_N=2*step_s./(v_N+v_N_1);
%ʱ�����
Cost_N=lambda_T*abs(t_N-t_exp)+Cost_J_N;
Matrix_Jmin(:,N)=Cost_N;
Matrix_Select(:,N)=1;

%% ���վ���һ����ת�Ƶ��յ�վǰһ����
for j=N-1:-1:2%��N-1��״̬ת�Ƶ��ڶ���״̬ת�ƣ����վ��һ����ת�Ƶ����վ�������㣩
%     
%      x=['��= ',num2str(j)];
%      disp(x)
     
    v_j=zeros(Speed_N+1,Speed_N+1);
    v_j_1=zeros(Speed_N+1,Speed_N+1);
    for i=1:1:Speed_N+1
        vi=step_v*(i-1)*ones(1,Speed_N+1);
        v_j(i,:)=vi;
        v_j_1(i,:)=vi;
    end
    v_j_1=v_j_1';%ת��
    
    aj_j_1=(v_j<=MaxCapacityV(j) & v_j_1<=MaxCapacityV(j+1)).*((v_j_1+v_j).*(v_j_1-v_j)/(2*step_s))+(v_j>MaxCapacityV(j) | v_j_1>MaxCapacityV(j+1)).*(1e16);%���ٶȾ���
    %״̬j�����п��ܵ��ٶȵ�ת�Ƶ�j+1״̬���п��ܵ��ٶȵ�ļ��ٶȵľ���
    %a(m,n)��ʾj״̬�ٶ�Ϊstep_v*(m-1) ת�Ƶ� j+1״̬�ٶ�Ϊtep_v*(n-1)����Ҫ�ļ��ٶ�
    
    R=GetBasicResistance(v_j*3.6)*M*g + wj(1,j)*1000;%����������������·������������
    Ft_max=GetTractionForce(v_j*3.6)*1000;%���ǣ�������󣬵�λN
    Fd_max=GetMaxBrakeForce(v_j*3.6)*1000;%����ƶ������󣬵�λN
    
    F_k=(aj_j_1>=a_min & aj_j_1<=a_max).*(aj_j_1*M*1000+R)+(aj_j_1>a_max |aj_j_1<a_min).*(1e32);
    %״̬j�����п��ܵ��ٶȵ�ת�Ƶ�j+1״̬���п��ܵ��ٶȵ� �� �г�ʩ������ ����
    
    Cost_J=(F_k>Ft_max|F_k<-Fd_max).*(1e32)+(F_k>=0 & F_k<=Ft_max).*(F_k*step_s/Eta)+(F_k<0 & F_k>=-Fd_max).*(F_k*step_s*alpha_Re*Eta);
    % ǣ���ܺĴ��۾���
    
    t_k=2*step_s./(v_j+v_j_1);
    %ʱ�����
    Cost=lambda_T*abs(t_k-t_exp)+Cost_J;
    CostT=Cost'+Matrix_Jmin(:,j+1);
    [value,index]=min(CostT);
    Matrix_Jmin(:,j)=value';
    Matrix_Select(:,j)=index';
    
end
%% �����վת�Ƶ����վ���һ����
v_1=zeros(Speed_N+1,1);
v_2=zeros(Speed_N+1,1);
for i=1:1:Speed_N+1
    vi=step_v*(i-1);
    v_1(i,1)=0;
    v_2(i,1)=vi;
end
a_1=(v_1<=MaxCapacityV(1) & v_2<=MaxCapacityV(2)).*((v_2+v_1).*(v_2-v_1)/(2*step_s))+(v_1>MaxCapacityV(1) | v_2>MaxCapacityV(2)).*(1e16);%���ٶȾ���
R_1=GetBasicResistance(v_1*3.6)*M*g + wj(1,1)*1000;%����������������·������������
Ft_max_1=GetTractionForce(v_1*3.6)*1000;%���ǣ�������󣬵�λN
Fd_max_1=GetMaxBrakeForce(v_1*3.6)*1000;%����ƶ������󣬵�λN
F_1=(a_1>=a_min & a_1<=a_max).*(a_1*M*1000+R_1)+(a_1>a_max |a_1<a_min).*(1e16);
Cost_J_1=(F_1>Ft_max_1|F_1<-Fd_max_1).*(1e16)+(F_1>=0 & F_1<=Ft_max_1).*(F_1*step_s/Eta)+(F_1<0 & F_1>=-Fd_max_1).*(F_1*step_s*alpha_Re*Eta);
t_1=2*step_s./(v_1+v_2);
%ʱ�����
Cost_1=lambda_T*abs(t_1-t_exp)+Cost_J_1;
Matrix_Jmin(:,1)=Cost_1;
Matrix_Select(:,1)=1:1:Speed_N+1;

%% ��Ѱ��һ������ת�ƴ�����С�ĵ㣬Ȼ��ͨ��Matrix_Select����������������ת��·���ϵ������ٶȵ�


[value,flag]=min(Matrix_Jmin(:,1));

s=zeros(1,N+1);%��ʼվ����Ϊ�����һ����
v=zeros(1,N+1);%��ʼվ����Ϊ�����һ����
F=zeros(1,N+1);%��ʼվ����Ϊ�����һ����
s(1,1)=start_pos;
v(1,1)=v0;
F(1,N+1)=0;
T=0;
E=0;
Et=0;
Eb=0;
T1=0;
for i=1:1:N
    s(1,i+1)=s(1,i)+step;
    flag=Matrix_Select(flag,i);
    v(1,i+1)=step_v*(flag-1);
    if s(1,i+1)>=12240
       T1=T1+2*step_s./(v(1,i)+v(1,i+1));
    end
    T=T+2*step_s./(v(1,i)+v(1,i+1));
    a=(v(1,i+1)^2-v(1,i)^2)/(2*step_s);
    F(1,i)=a*M*1000+GetBasicResistance(v(1,i)*3.6)*M*g + wj(1,i)*1000;
    E=E+( F(1,i)>=0).*(F(1,i)*step_s/Eta)+(F(1,i)<0).*(F(1,i)*alpha_Re*Eta*step_s);
    Et=Et+( F(1,i)>=0).*(F(1,i)*step_s/Eta)+(F(1,i)<0).*(0);
    Eb=Eb+( F(1,i)>=0).*(0)+(F(1,i)<0).*(F(1,i)*alpha_Re*Eta*step_s);
end
t=toc;
x=['ʱ���ѹ�',num2str(t), ' s'];
disp(x)
end
