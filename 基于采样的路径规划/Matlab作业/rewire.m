function T = rewire(T, X_nears, x_new, Imp)
% 对于除了最近节点x_near外的邻近节点来说 若通过x_new使其到起点的距离更短，更新x_new作为它的父节点，
count = size(X_nears, 2);
new_idx = size(T.v, 2);

for i = 2:count
    pre_cost = T.v(X_nears(i)).dist; 
    near_node(1) = T.v(X_nears(i)).x;
    near_node(2) = T.v(X_nears(i)).y;
    tentative_cost = distance(near_node, x_new) + T.v(new_idx).dist; % 通过x_new到起点的cost
    if ~collisionChecking(near_node, x_new,Imp)  % rewire过程中也要碰撞检测
        continue;
    end
    if tentative_cost<pre_cost  % 若通过起点使cost降低，改变其父节点为x_new
        T.v(X_nears(i)).xPrev = x_new(1);
        T.v(X_nears(i)).yPrev = x_new(2);
        T.v(X_nears(i)).dist = tentative_cost;
        T.v(X_nears(i)).indPrev = new_idx;
    end
end

end