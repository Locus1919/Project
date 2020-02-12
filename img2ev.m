function [E, G] = img2ev(x)
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明
img = imread (x);
imgedge = edge(img,'Prewitt',0.094);
[a,b] = size(imgedge);
flag = 1;
for i = 1:a
    for j = 1:b
        if imgedge(i,j) == 1
            E(flag, 1) = i;
            E(flag, 2) = j;
            flag = flag + 1;
        end
    end
end
[Gx,Gy] = imgradientxy(img);
length(E)
for i = 1 : length(E)
    G(i,1) = Gx(E(i,1),E(i,2));
    G(i,2) = Gy(E(i,1),E(i,2));
end
end

