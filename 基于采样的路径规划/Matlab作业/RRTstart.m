%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% 初始化
clc
clear all; close all;
x_I=1; y_I=1;           % 设置初始点
x_G=700; y_G=700;       % 设置目标点
% x_G=327; y_G=285;       % 设置目标点
% x_G=760; y_G=500;  % narrow passage

Thr=50;                 % 设置目标点阙值(离目标点附近多远被视为终点)
Delta= 30;              % 设置扩展步长
%%建树初始化
T.v(1).x = x_I;         % T是树，v是节点，先把起始点加入树中
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     %起始节点的父节点依然是其本身
T.v(1).yPrev = y_I;
T.v(1).dist=0;          % 父节点到该节点的距离，可取欧氏距离
T.v(1).indPrev = 0;     %父节点的index
%% 
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp) %800*800
xL=size(Imp,2);%地图x轴长度
yL=size(Imp,1);%地图y轴长度
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% 绘制起点和目标点
bFind = false;

min_path_cost = 10000;
this_path = plot([0,1],[1,1]); % 创建一个线段句柄，后续用于生成路径的储存和删除
pic_num = 1;
tic; % tic, toc计时

for iter = 1:800
    
    %Step 1:在地图中随机采样一个点x_rand
    %提示用（x_rand(1), x_rand(2)）表示环境中采样点的坐标
%     if bFind
%         x_rand = randi(800, 1, 2);  % 找到路径后 全局随机采样，优化路径
%     else
%         x_rand = Sample(iter, xL, yL, x_G, y_G);  % 先全局后重点采样，快速生成路径
%     end

    x_rand = randi(800, 1, 2);  % 找到路径后 全局随机采样，优化路径

%     % 用蓝色实心点表示采样点
    plot(x_rand(1), x_rand(2), 'ro', 'MarkerSize',4, 'MarkerFaceColor','b');


    %Step 2: 遍历树，从树中找到最近邻近点x_near
    %提示：x_near已经在树T里
    [x_near, near_Idx] =Near(x_rand, T);

    %Step 3: 拓展得到x_new节点, 相似三角形
    %注意使用拓展步长Delta
    x_new=Steer(x_rand, x_near, Delta);
    plot(x_new(1), x_new(2), 'ro', 'MarkerSize',2);
    
    %检查节点是否是collision free
    if ~collisionChecking(x_near,x_new,Imp) 
        continue;
    end
    % 找到x_new的临近节点
    X_nears = NearC(T, x_new, near_Idx);
    % 找到使得x_new到起点cost最小的父节点x_min
    [x_min, min_Idx] = ChooseParent(X_nears, x_new, T);
%  将x_min作为x_new的父节点加入树中
    T =  AddNode(T, x_new, x_min, min_Idx);
%   更新临近节点的父节点
    T = rewire(T, X_nears, x_new, Imp);
    
    
    X = [x_new(1), x_min(1)];
    Y = [x_new(2), x_min(2)];
%     x_steer = [x_new(1), x_rand(1)];
%     y_steer = [x_new(2), x_rand(2)];
    plot(X, Y, '-b');
    hold on; 
    
    %Step 5:检查是否达到目标点附近
    %提示：x_new， x_G之间的距离是否小于Thr，小于则跳出for
    if bFind == false    % 判断是否已经找到路径
        dis_goal = sqrt(power(x_new(1)-x_G,2) + power(x_new(2) - y_G, 2) );
        if dis_goal<Thr
            bFind = true;
            end_Idx = T.v(end).indPrev;  %记录此时的索引值
%         break;  % 继续采样优化所得路径
        end
    else  % 找到路径则不断更新路径情况
        path.pos(1).x = x_G;
        path.pos(1).y = y_G;
        path.pos(2).x = T.v(end_Idx).x; path.pos(2).y = T.v(end_Idx).y;
        

        path_cost = sqrt(power(path.pos(1).x-path.pos(2).x ,2) + power(path.pos(1).y  - path.pos(2).y, 2)) + T.v(end_Idx).dist;
        pathIndex = T.v(end_Idx).indPrev; % 终点索引值
        
        if path_cost<min_path_cost %找到更短路径
            min_path_cost = path_cost;
            j=0;
            while 1 % 构建路径
                path.pos(j+3).x = T.v(pathIndex).x;
                path.pos(j+3).y = T.v(pathIndex).y;
                pathIndex = T.v(pathIndex).indPrev;

                if pathIndex == 1
                    break
                end
                j=j+1;
            end  % 由终点回溯到起点
            path.pos(end+1).x = x_I; path.pos(end).y = y_I; % 起点加入路径
            path_X = [];
            path_Y = [];
            for j = 1:length(path.pos)
                path_X(j) = path.pos(j).x;
                path_Y(j) = path.pos(j).y;
            end
            delete(this_path); %删除旧路径
            this_path = plot(path_X, path_Y, 'b', 'Linewidth', 3); % 生成新路径
            fprintf('\n Path cost is %.2f\n Sampling nodes: %d\n Tree nodes: %d\n',path_cost, iter, size(T.v,2))
        end
    end
    
    %Step 6:将x_near和x_new之间的路径画出来
    %提示1：使用plot绘制，需要多次绘制，加上hold on
    %提示2：判断终止条件满足前，先将x_near和x_new之间的路径画出来


%     pause(0.01); %暂停一会，使得rrt扩展容易观察
    
    % 生成gif动图
    F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);

    if pic_num == 1
    imwrite(I,map,'test.gif','gif','Loopcount',inf,'DelayTime',0);

    else
    imwrite(I,map,'test.gif','gif','WriteMode','append','DelayTime',0);

    end

    pic_num = pic_num + 1;
end
toc
%% 路径已经找到，反向查询
