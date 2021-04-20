function T = AddNode(T, x_new, x_near, near_Idx)
% 将collision_free的x_new节点加入树T中，并以x_near为父节点
    count = size(T.v,2) + 1;
    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2);
    T.v(count).xPrev = T.v(near_Idx).x;
    T.v(count).yPrev = T.v(near_Idx).y;
    T.v(count).dist=distance(x_new, x_near) + T.v(near_Idx).dist;  % 该节点到原点的距离
    T.v(count).indPrev = near_Idx;     %父节点的index
end
