function [s,v,F,T,E,Matrix_Jmin]=DynamicProgram()
%% 输出
% s: 离散的空间轴，单位m
% v：优化后的速度曲线，单位km/h
% F: 列车输出力在空间域上的离散序列，单位N
% T：列车总运行时间，单位s
% E：考虑再生制动的列车总牵引能耗,单位J

%% 自底而上的常规动态规划求解方法
global Speed_N;
global N;
global step_s;
global step_v;
global v0;
global vend;
global start_pos;
global end_pos;
global MaxCapacityV;%1:N+1,最大能力速度曲线
if end_pos>start_pos
    step=step_s;
else
   step=-step_s;
end

Matrix_Jmin=inf*ones(Speed_N+1,N);%回溯记录矩阵 之 状态转移代价累计矩阵

%第i行表示(i-1)Δv的速度，第j列表示空间域上第j个点转移到终点（第N+1个点）所需要的最小的总代价Jmin
%（i,j）表示从空间域上第j个点，以速度(i-1)Δv转移到终点（第N+1个点）所需要的最小的总代价Jmin
%j=1:N，j=1表示从起始站转移到起始站后一个点

Matrix_T=zeros(Speed_N+1,N);%回溯记录矩阵 之 状态转移耗时累计矩阵
Matrix_F=zeros(Speed_N+1,N);%回溯记录矩阵 之 状态转移所需列车输出力矩阵
Matrix_E=zeros(Speed_N+1,N);%回溯记录矩阵 之 状态转移能耗累计矩阵
Matrix_Select=ones(Speed_N+1,N);%回溯记录矩阵 之 状态转移最优路径选择矩阵

%% 终点站前一个点转移到终点站
[L,U]=RangeDecrease(N,0,0);
for i=L:1:U%最后一列不需要选择代价最小路径，因为终点站速度为0，终点站不存在多种速度状态点
    [J_k,t_k,F_k,e_k]=RecurrenceEquation(step_v*(i-1),0,N);
    Matrix_Jmin(i,N)=J_k;
    Matrix_T(i,N)=t_k;
    Matrix_F(i,N)=F_k;
    Matrix_E(i,N)=e_k;
    Matrix_Select(i,N)=1;%不用选择所以等于0
end
%disp(Matrix_Jmin(:,N));

%% 起点站后第一个点转移到终点站前一个点
for j=N-1:-1:2%从N-1次状态转移到第二次状态转移（起点站后一个点转移到起点站后两个点）
    L_pre=L;
    U_pre=U;
    disp(L)
    disp(U)
    [L,U]=RangeDecrease(j,L,U);%求j+1状态转移到j状态的点的可能范围
    for i=1:1:U
        x=['列= ',num2str(j),'行= ',num2str(i)];
        disp(x);
        if MaxCapacityV(j+1)==0 %求j状态的i点转移到j+1状态的点的可能范围
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
        for z=L3:1:U3%筛选出对于第j个点（j=N-1:1:2）,速度为strp_v*(i-1)的点转移到终点的最优转移路径
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
        Matrix_Select(i,j)=select;%select意味着转移到速度为strp_v*(select-1)的点上去
    end
end
%% 从起点站转移到起点站后第一个点
for i=1:1:Speed_N+1
    [J_k,t_k,F_k,e_k]=RecurrenceEquation(0,step_v*(i-1),1);
    Matrix_Jmin(i,1)=J_k+Matrix_Jmin(i,2);
    Matrix_T(i,1)=t_k+Matrix_T(i,2);
    Matrix_F(i,1)=F_k;
    Matrix_E(i,1)=e_k+Matrix_E(i,2);
    Matrix_Select(i,1)=i;
end
%% 搜寻第一列中总转移代价最小的点，然后通过Matrix_Select矩阵正向连结最优转移路径上的所有速度点

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
s=zeros(1,N+1);%起始站点作为矩阵第一个点
v=zeros(1,N+1);%起始站点作为矩阵第一个点
F=zeros(1,N+1);%起始站点作为矩阵第一个点
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
