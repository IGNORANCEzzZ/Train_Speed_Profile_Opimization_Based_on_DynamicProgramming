function [SpdLimit]=GetSpeedLimit(IsStop)
%% Outputs:
% SpdLimit: Speed limit, unit: km/h
% SpdLimit: 1:N, starting station next point as beginning
%% Input:
% IsStop: Whether to stop at intermediate stations

%% 
 global SpeedLimit;
 global Station;
 global start_pos;
 global end_pos;
 global N;
 global step_s;
 
 cur_pos=start_pos;
 Speed=SpeedLimit;
 [row_staion,col_station]=size(Station);
%% Get track speed limit information
 SpdLimit=zeros(1,N);

 if end_pos>start_pos
     [row,col]=size(Speed);
     for i=1:1:row
         if start_pos>=Speed(i,1) &&start_pos<Speed(i,3)
             break;
         end
     end
     start_flag=i;
     for j=1:1:N
         cur_pos=cur_pos+step_s;
         if cur_pos>end_pos % Handle end section
             cur_pos=end_pos;
         end
         while(1)
             if cur_pos>=Speed(start_flag,1) &&cur_pos<Speed(start_flag,3)
                 SpdLimit(1,j)=Speed(start_flag,2);
                 break;
             else
                 start_flag=start_flag+1;
             end
         end
         if IsStop
             for i=1:1:row_staion
                 if (cur_pos==Station(i,1) && j~=N )||(cur_pos<Station(i,1) && cur_pos+step_s>Station(i,1)&& j~=N-1)
                     SpdLimit(1,j)=0;
                 end
             end
         end
     end
 else
     [row,col]=size(Speed);
     for i=1:1:row
         if start_pos>=Speed(i,1) &&start_pos<Speed(i,3)
             break;
         end
     end
     start_flag=i;
     for j=1:1:N
         cur_pos=cur_pos-step_s;
         if cur_pos<end_pos % Handle end section
             cur_pos=end_pos;
         end
         while(1)
             if cur_pos>=Speed(start_flag,1) &&cur_pos<Speed(start_flag,3)
                 SpdLimit(1,j)=Speed(start_flag,2);
                 break;
             else
                 start_flag=start_flag-1;
             end
         end
         if IsStop
             for i=1:1:row_staion
                 if (cur_pos==Station(i,1) && j~=N )||(cur_pos<Station(i,1) && cur_pos+step_s>Station(i,1)&& j~=N-1)
                     SpdLimit(1,j)=0;
                 end
             end
         end
     end
 end
end