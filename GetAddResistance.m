function [wj,gradient,curve]=GetAddResistance()
%% 输出参数
% wj：附加阻力，单位kN
% wj，1：N，以起始站后一个点为初始点
%%
global Gradient;
global Curve;
 global start_pos;
 global end_pos;
 global N;
 global step_s;
 global g;
 global M;
 cur_pos=start_pos;
 
 %% 得到线路坡度矩阵
 gradient=zeros(1,N);
 if end_pos>start_pos
       [row,col]=size(Gradient);
     for i=1:1:row
         if start_pos>=Gradient(i,1) &&start_pos<Gradient(i,3)
             break;
         end
     end
     start_flag=i;
     for j=1:1:N
         cur_pos=cur_pos+step_s;
         disp(cur_pos)
         while(1)
             disp(start_flag)
             disp(Gradient(start_flag,1))
             disp(Gradient(start_flag,3))
             
         if cur_pos>=Gradient(start_flag,1) &&cur_pos<Gradient(start_flag,3)
             gradient(1,j)=Gradient(start_flag,2);
             break;
         else 
             start_flag=start_flag+1;
         end
         end
     end
 else
     [row,col]=size(Gradient);
     for i=1:1:row
         if start_pos>=Gradient(i,1) &&start_pos<Gradient(i,3)
             break;
         end
     end
     start_flag=i;
     for j=1:1:N
         cur_pos=cur_pos-step_s;
         while(1)
         if cur_pos>=Gradient(start_flag,1) &&cur_pos<Gradient(start_flag,3)
             gradient(1,j)=-Gradient(start_flag,2);
             break;
         else 
             start_flag=start_flag-1;
         end
         end
     end
 end
 %% 得到线路曲线矩阵
  curve=zeros(1,N);
  cur_pos=start_pos;
 if end_pos>start_pos
       [row,col]=size(Curve);
     for i=1:1:row
         if start_pos>=Curve(i,1) &&start_pos<Curve(i,3)
             break;
         end
     end
     start_flag=i;
     for j=1:1:N
         cur_pos=cur_pos+step_s;
         while(1)
         if cur_pos>=Curve(start_flag,1) &&cur_pos<Curve(start_flag,3)
             curve(1,j)=Curve(start_flag,2);
             break;
         else 
             start_flag=start_flag+1;
         end
         end
     end
 else
     [row,col]=size(Curve);
     for i=1:1:row
         if start_pos>=Curve(i,1) &&start_pos<Curve(i,3)
             break;
         end
     end
     start_flag=i;
     for j=1:1:N
         cur_pos=cur_pos-step_s;
         while(1)
         if cur_pos>=Curve(start_flag,1) &&cur_pos<Curve(start_flag,3)
             curve(1,j)=Curve(start_flag,2);
             break;
         else 
             start_flag=start_flag-1;
         end
         end
     end
 end
 
 %% 得到单位附加阻力矩阵 单位N/KN
 wj=zeros(1,N);
 for i=1:1:N
     if curve(1,i)==0
         wj(1,i)=gradient(1,i)*M*g*10^-3;
     else
         wj(1,i)=(gradient(1,i)+600/curve(1,i))*M*g*10^-3;%单位kN
     end
 end
end

