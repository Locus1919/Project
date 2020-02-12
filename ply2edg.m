function edgepoint = ply2edg(x)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
a = pcread(x);
b = copy(a);
c = pcmerge(a,b,0.05);
edgepoint = c.Location;
end

