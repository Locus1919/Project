function edgepoint = ply2edg(x)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
a = pcread(x);
b = copy(a);
c = pcmerge(a,b,0.05);
edgepoint = c.Location;
end

