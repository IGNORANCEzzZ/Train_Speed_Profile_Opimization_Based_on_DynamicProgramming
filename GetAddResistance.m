function [wj,gradient,curve]=GetAddResistance()
%% �������
% wj��������������λkN
% wj��1��N������ʼվ��һ����Ϊ��ʼ��
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
 
 %% �õ���·�¶Ⱦ���
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
 %% �õ���·���߾���
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
 
 %% �õ���λ������������ ��λN/KN
 wj=zeros(1,N);
 for i=1:1:N
     if curve(1,i)==0
         wj(1,i)=gradient(1,i)*M*g*10^-3;
     else
         wj(1,i)=(gradient(1,i)+600/curve(1,i))*M*g*10^-3;%��λkN
     end
 end
end

