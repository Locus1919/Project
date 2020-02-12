function [u,v] = tdp(i,P,filename)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
groundtruth = csvread(filename,3,4,[3,4,3,7]);
tran = csvread(filename,3,1,[3,1,3,3]);
rotm = quat2rotm(groundtruth);
a = P(i,:) - tran;
b = rotm';
c = a * b;
K = [458.654,0,367.215;
    0,457.296,248.375;
    0,0,1];
pc = K * c';
u = pc(1,1) / pc(3,1);
v = pc(2,1) / pc(3,1);
end

