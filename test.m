clear; close all; clc;
img = imread('E:/lidar/mav0/cam0/data/1403715274312143104.png');
[h,w] = size(img);
[E, G] = img2ev('E:/lidar/mav0/cam0/data/1403715274312143104.png');
P = ply2edg('E:/lidar/mav0/result/edges-50¡¢0.05.ply');
Tree = KDTreeSearcher (E);
flag = 1;
for i = 1 : length(P)
    [u,v] = tdp(i,P,'E:/lidar/mav0/state_groundtruth_estimate0/data.csv');
    if u > 0 && u < w && v > 0 && v < h
        Ppoints(flag,1) = u;
        Ppoints(flag,2) = v;
        flag = flag + 1;
    end
end
[search,dis] = knnsearch(Tree,Ppoints);
best_dis = 0;
for i = 1:length(dis)
    best_dis = best_dis + dis(i,1);
end
for i = 1 : length(search)
    matches(i,1) = E(search(i,1),1);
    matches(i,2) = E(search(i,1),2);
end
best_R = 0;
best_t = 0;
groundtruth = csvread('E:/lidar/mav0/state_groundtruth_estimate0/data.csv',3,4,[3,4,3,7]);
t = csvread('E:/lidar/mav0/state_groundtruth_estimate0/data.csv',3,1,[3,1,3,3]);
R = quat2rotm(groundtruth);
dt = [-0.01,0,0.01];
dR = [-2.5,0,2.5];
for a = 1:3
    dx = dt(1,a);
    for b = 1:3
        dy = dt(1,b);
        for c = 1:3
            dz = dt(1,c);
            for d = 1:3
                dtx = dR(1,d);
                for e = 1:3
                    dty = dR(1,e);
                    for f = 1:3
                        dtz = dR(1,f);
                        Dt = [dx,dy,dz];
                        for g = 1:3
                            new_t(1,g) = t(1,g) + Dt(1,g);
                        end
                        DR = eul2rotm([dtx,dty,dtz]);
                        new_R = R*DR;
                        flagg = 1;
                        for i = 1 : length(P)
                            [u,v] = tdp2(new_R,new_t,i,P);
                            if u > 0 && u < w && v > 0 && v < h
                                Propoints(flagg,1) = u;
                                Propoints(flagg,2) = v;
                                flagg = flagg + 1;
                            end
                        end
                        totaldis = 0;
                        for j = 1 : length(matches)
                            distance = 0;
                            for k = 1 : 2
                                distance = distance + (matches(j,k)-Propoints(j,k))^2;
                            end
                            distance = sqrt(distance);
                            totaldis = totaldis + distance;
                        end
                        if totaldis <= best_dis
                            best_dis = totaldis;
                            best_R = new_R;
                            best_t = new_t;
                        end
                    end
                end
            end
        end
    end
end
