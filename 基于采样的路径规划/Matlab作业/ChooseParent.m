function [x_min, min_idx] = ChooseParent(X_nears, x_new, T)
% 在邻近集合X_nears中，找到使得x_new通往起点距离最短的父节点
nearest = [];
min_cost = 10000; %计算x_new->nearest node->起点需要的cost
count = size(X_nears, 2);

for i = 1:count
    nearest(1) = T.v(X_nears(i)).x;
    nearest(2) = T.v(X_nears(i)).y;
    cost = distance(nearest, x_new) + T.v(X_nears(i)).dist; 
    if cost<min_cost
        min_cost = cost;
        x_min = nearest;
        min_idx = X_nears(i);
    end
end
end
    