function [ output_args ] = GetBasicResistance( speed )
%�����г����л�������
%���������
    %speed���г������ٶ�,km/h
%���������
    %���� ��λ��N/KN(ţÿǧţ)
a=2.031;
b=0.0622;
c=0.001807;
output_args=a+b*speed+c*speed*speed;
end
