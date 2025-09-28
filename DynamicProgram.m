function [s,v,F,T,E,Matrix_Jmin]=DynamicProgram()
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
global vend;
global start_pos;
global end_pos;
global MaxCapacityV;%1:N+1,��������ٶ�����
if end_pos>start_pos
    step=step_s;
else
   step=-step_s;
end

Matrix_Jmin=inf*ones(Speed_N+1,N);%���ݼ�¼���� ֮ ״̬ת�ƴ����ۼƾ���

%��i�б�ʾ(i-1)��v���ٶȣ���j�б�ʾ�ռ����ϵ�j����ת�Ƶ��յ㣨��N+1���㣩����Ҫ����С���ܴ���Jmin
%��i,j����ʾ�ӿռ����ϵ�j���㣬���ٶ�(i-1)��vת�Ƶ��յ㣨��N+1���㣩����Ҫ����С���ܴ���Jmin
%j=1:N��j=1��ʾ����ʼվת�Ƶ���ʼվ��һ����

Matrix_T=zeros(Speed_N+1,N);%���ݼ�¼���� ֮ ״̬ת�ƺ�ʱ�ۼƾ���
Matrix_F=zeros(Speed_N+1,N);%���ݼ�¼���� ֮ ״̬ת�������г����������
Matrix_E=zeros(Speed_N+1,N);%���ݼ�¼���� ֮ ״̬ת���ܺ��ۼƾ���
Matrix_Select=ones(Speed_N+1,N);%���ݼ�¼���� ֮ ״̬ת������·��ѡ�����

%% �յ�վǰһ����ת�Ƶ��յ�վ
[L,U]=RangeDecrease(N,0,0);
for i=L:1:U%���һ�в���Ҫѡ�������С·������Ϊ�յ�վ�ٶ�Ϊ0���յ�վ�����ڶ����ٶ�״̬��
    [J_k,t_k,F_k,e_k]=RecurrenceEquation(step_v*(i-1),0,N);
    Matrix_Jmin(i,N)=J_k;
    Matrix_T(i,N)=t_k;
    Matrix_F(i,N)=F_k;
    Matrix_E(i,N)=e_k;
    Matrix_Select(i,N)=1;%����ѡ�����Ե���0
end
%disp(Matrix_Jmin(:,N));

%% ���վ���һ����ת�Ƶ��յ�վǰһ����
for j=N-1:-1:2%��N-1��״̬ת�Ƶ��ڶ���״̬ת�ƣ����վ��һ����ת�Ƶ����վ�������㣩
    L_pre=L;
    U_pre=U;
    disp(L)
    disp(U)
    [L,U]=RangeDecrease(j,L,U);%��j+1״̬ת�Ƶ�j״̬�ĵ�Ŀ��ܷ�Χ
    for i=1:1:U
        x=['��= ',num2str(j),'��= ',num2str(i)];
        disp(x);
        if MaxCapacityV(j+1)==0 %��j״̬��i��ת�Ƶ�j+1״̬�ĵ�Ŀ��ܷ�Χ
            L3=1;
            U3=1;
        else
            [L2,U2]=RangeDecrease(j+1,i,i);
            L3=min(L_pre,L2);
            U3=max(U_pre,U2);
        end
        z=L3;
        [J_k,t_k,F_k,e_k]=RecurrenceEquation(step_v*(i-1),step_v*(z-1),j);
        J_selected=J_k+Matrix_Jmin(z,j+1);
        t_selected=t_k+Matrix_T(z,j+1);
        F_selected=F_k;
        E_selected=e_k+Matrix_E(z,j+1);
        select=z;
        for z=L3:1:U3%ɸѡ�����ڵ�j���㣨j=N-1:1:2��,�ٶ�Ϊstrp_v*(i-1)�ĵ�ת�Ƶ��յ������ת��·��
            [J_k,t_k,F_k,e_k]=RecurrenceEquation(step_v*(i-1),step_v*(z-1),j);
            if J_k+Matrix_Jmin(z,j+1)<J_selected
                J_selected=J_k+Matrix_Jmin(z,j+1);
                t_selected=t_k+Matrix_T(z,j+1);
                F_selected=F_k;
                E_selected=e_k+Matrix_E(z,j+1);
                select=z;
            end
        end
        Matrix_Jmin(i,j)=J_selected;
        Matrix_T(i,j)=t_selected;
        Matrix_F(i,j)=F_selected;
        Matrix_E(i,j)=E_selected;
        Matrix_Select(i,j)=select;%select��ζ��ת�Ƶ��ٶ�Ϊstrp_v*(select-1)�ĵ���ȥ
    end
end
%% �����վת�Ƶ����վ���һ����
for i=1:1:Speed_N+1
    [J_k,t_k,F_k,e_k]=RecurrenceEquation(0,step_v*(i-1),1);
    Matrix_Jmin(i,1)=J_k+Matrix_Jmin(i,2);
    Matrix_T(i,1)=t_k+Matrix_T(i,2);
    Matrix_F(i,1)=F_k;
    Matrix_E(i,1)=e_k+Matrix_E(i,2);
    Matrix_Select(i,1)=i;
end
%% ��Ѱ��һ������ת�ƴ�����С�ĵ㣬Ȼ��ͨ��Matrix_Select����������������ת��·���ϵ������ٶȵ�

J_min=Matrix_Jmin(1,1);
flag=1;
for i=2:1:Speed_N+1
    if Matrix_Jmin(i,1)<J_min
        J_min=Matrix_Jmin(i,1);
        flag=i;
    end
end

T=Matrix_T(flag,1);
E=Matrix_E(flag,1);
s=zeros(1,N+1);%��ʼվ����Ϊ�����һ����
v=zeros(1,N+1);%��ʼվ����Ϊ�����һ����
F=zeros(1,N+1);%��ʼվ����Ϊ�����һ����
s(1,1)=start_pos;
v(1,1)=v0;
F(1,N+1)=0;
disp(Matrix_Jmin(:,N));
% disp(Matrix_Jmin(:,N-1));
% disp(Matrix_Jmin(:,N-2));
% disp(Matrix_Select(:,N));
% disp(Matrix_Select(:,N-1));
% disp(Matrix_Select(:,N-2));

for i=1:1:N
    s(1,i+1)=s(1,i)+step;
    flag=Matrix_Select(flag,i);
    v(1,i+1)=step_v*(flag-1);
    F(1,i)=Matrix_F(flag,i);
end
end
