function [ u,v ] = tdp2( R,t,i,P )
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
a = P(i,:) - t;
b = R';
c = a * b;
K = [458.654,0,367.215;
    0,457.296,248.375;
    0,0,1];
pc = K * c';
u = pc(1,1) / pc(3,1);
v = pc(2,1) / pc(3,1);
end

