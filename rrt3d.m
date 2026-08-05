% 3D RRT with Adaptive Energy Cost & Height-biased Sampling
% 特性：标准RRT + 自适应爬升惩罚(相对目标高度) + 高度偏置采样 + 膨胀障碍物 + 迭代剪枝 + B样条平滑
clear; close all; clc;

%% ================= 参数设置 =================
goal_bias        = 0.1;       % 目标偏置概率
height_bias_prob = 0.3;       % 高度偏置概率（使采样高度靠近目标）
step_size        = 2.0;       % 扩展步长
max_iter         = 4000;      % 最大迭代次数
goal_threshold   = 3.0;       % 目标距离阈值
spline_sample    = 200;       % B样条采样点数
inflate_factor   = 1.1;       % 障碍物膨胀系数
prune_max_passes = 10;        % 剪枝最大迭代次数

% 能量代价基础权重
energy_w_dist   = 1.0;        % 水平距离权重
energy_w_climb  = 0.5;        % 基础爬升权重（将根据高度差自适应调整）
energy_w_turn   = 0.5;        % 转向权重（仅用于最终评估，不加入RRT代价）

% 自适应爬升参数
adapt_scale     = 30.0;       % 高度差尺度（环境高度范围约100，取30）

% 空间边界
x_range = [0, 100];  y_range = [0, 100];  z_range = [0, 100];

% 起点与终点
start = [10, 10, 10];
goal  = [90, 90, 50];

% 原始长方体障碍物（底部贴地）
obstacles_original = [
    20, 30, 20, 40,  0, 30;
    50, 65, 40, 60,  0, 40;
    70, 80, 70, 85,  0, 50;
    15, 25, 60, 80,  0, 60;
    75, 90, 15, 30,  0, 55;
    ];

% 生成膨胀障碍物（用于碰撞检测）
obstacles_inflated = zeros(size(obstacles_original));
for i = 1:size(obstacles_original,1)
    b = obstacles_original(i,:);
    cx = (b(1)+b(2))/2;  cy = (b(3)+b(4))/2;  cz = (b(5)+b(6))/2;
    hx = (b(2)-b(1))/2;  hy = (b(4)-b(3))/2;  hz = (b(6)-b(5))/2;
    obstacles_inflated(i,:) = [cx - inflate_factor*hx, cx + inflate_factor*hx, ...
                               cy - inflate_factor*hy, cy + inflate_factor*hy, ...
                               cz - inflate_factor*hz, cz + inflate_factor*hz];
end
obstacles_inflated(:,5) = max(obstacles_inflated(:,5), 0);

%% ================= 初始化 RRT 树 =================
nodes  = start;
cost   = 0;            % 累积能量代价
parent = 0;

% rng(42);

%% ================= RRT 主循环 =================
goal_reached = false;
goal_idx     = 0;

for iter = 1:max_iter
    % 1. 随机采样（目标偏置 + 高度偏置）
    if rand < goal_bias
        sample = goal;
    else
        sample = [ x_range(1)+rand*(x_range(2)-x_range(1)), ...
                   y_range(1)+rand*(y_range(2)-y_range(1)), ...
                   z_range(1)+rand*(z_range(2)-z_range(1)) ];
        if rand < height_bias_prob
            sample(3) = goal(3) + (rand-0.5)*10;
            sample(3) = max(z_range(1), min(z_range(2), sample(3)));
        end
    end

    % 2. 查找最近节点（使用自适应能量距离）
    nearest_idx = findNearestAdaptive(nodes, sample, goal(3), energy_w_dist, energy_w_climb, adapt_scale);
    nearest_node = nodes(nearest_idx, :);

    % 3. 向采样方向扩展
    direction = sample - nearest_node;
    dir_norm  = norm(direction);
    if dir_norm < 1e-6, continue; end
    new_node = nearest_node + (step_size / dir_norm) * direction;

    % 4. 边界检查
    if new_node(1) < x_range(1) || new_node(1) > x_range(2) || ...
       new_node(2) < y_range(1) || new_node(2) > y_range(2) || ...
       new_node(3) < z_range(1) || new_node(3) > z_range(2)
        continue;
    end

    % 5. 碰撞检测
    if checkCollision(nearest_node, new_node, obstacles_inflated)
        continue;
    end

    % 6. 计算自适应能量增量
    delta_energy = energyIncrementAdaptive(nearest_node, new_node, goal(3), energy_w_dist, energy_w_climb, adapt_scale);
    new_cost = cost(nearest_idx) + delta_energy;

    % 7. 加入树
    nodes  = [nodes; new_node];
    cost   = [cost; new_cost];
    parent = [parent; nearest_idx];
    new_idx = size(nodes, 1);

    % 8. 检测目标
    if ~goal_reached && norm(new_node - goal) <= goal_threshold
        if ~checkCollision(new_node, goal, obstacles_inflated)
            goal_inc = energyIncrementAdaptive(new_node, goal, goal(3), energy_w_dist, energy_w_climb, adapt_scale);
            goal_energy = new_cost + goal_inc;
            nodes  = [nodes; goal];
            cost   = [cost; goal_energy];
            parent = [parent; new_idx];
            goal_idx = size(nodes, 1);
            goal_reached = true;
            break;
        end
    end
end

%% ================= 提取路径 =================
if ~goal_reached
    error('在最大迭代次数内未找到路径。');
end

path_indices = [];
idx = goal_idx;
while idx ~= 0
    path_indices = [idx; path_indices];
    idx = parent(idx);
end
original_path = nodes(path_indices, :);

%% ================= 路径剪枝（基于膨胀障碍物） =================
pruned_path = iterativePrune(original_path, obstacles_inflated, prune_max_passes);

%% ================= B样条平滑 =================
smooth_candidate = bsplineSmooth(pruned_path, 4, spline_sample);
if isPathCollisionFree(smooth_candidate, obstacles_inflated)
    final_path = smooth_candidate;
    use_bspline = true;
else
    final_path = pruned_path;
    use_bspline = false;
end

%% ================= 能量消耗评估（仍使用固定爬升权重，便于对比） =================
fprintf('\n========== 能量消耗评估 (固定权重: 水平=%.1f, 爬升=%.1f, 转向=%.2f) ==========\n', ...
        energy_w_dist, energy_w_climb, energy_w_turn);
E_orig = computeEnergy(original_path, energy_w_dist, energy_w_climb, energy_w_turn);
fprintf('原始路径  : 总能耗=%.2f (水平=%.2f, 爬升=%.2f, 转向=%.2f rad)\n', ...
        E_orig.total, E_orig.dist, E_orig.climb, E_orig.turn_sum);
E_prune = computeEnergy(pruned_path, energy_w_dist, energy_w_climb, energy_w_turn);
fprintf('迭代剪枝后: 总能耗=%.2f (水平=%.2f, 爬升=%.2f, 转向=%.2f rad)\n', ...
        E_prune.total, E_prune.dist, E_prune.climb, E_prune.turn_sum);
E_final = computeEnergy(final_path, energy_w_dist, energy_w_climb, energy_w_turn);
if use_bspline
    fprintf('B样条平滑后: 总能耗=%.2f (水平=%.2f, 爬升=%.2f, 转向=%.2f rad)\n', ...
            E_final.total, E_final.dist, E_final.climb, E_final.turn_sum);
else
    fprintf('最终路径(剪枝): 总能耗=%.2f (水平=%.2f, 爬升=%.2f, 转向=%.2f rad)\n', ...
            E_final.total, E_final.dist, E_final.climb, E_final.turn_sum);
end
fprintf('==============================================\n\n');

%% ================= 三维可视化 =================
figure('Name', '3D RRT (Adaptive Climb Cost)', 'Position', [100,100,900,600]);
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title(['3D RRT 自适应爬升惩罚 (最终能耗=' num2str(E_final.total, '%.1f') ')']);
view(3);

% 障碍物
plotAABBObstacles(obstacles_original);
for i = 1:size(obstacles_inflated,1)
    drawInflatedBBox(obstacles_inflated(i,:));
end

% RRT 树
plot3(NaN, NaN, NaN, 'Color', [0.7 0.85 1], 'LineWidth', 0.3, 'DisplayName', 'RRT 树');
for i = 2:size(nodes,1)
    p1 = nodes(parent(i),:);
    p2 = nodes(i,:);
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], ...
          'Color', [0.7 0.85 1], 'LineWidth', 0.3, 'HandleVisibility', 'off');
end

plot3(start(1),start(2),start(3), 'go', 'MarkerSize',10, 'MarkerFaceColor','g', 'DisplayName', '起点');
plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r', 'DisplayName', '终点');

plot3(original_path(:,1), original_path(:,2), original_path(:,3), ...
      'b--', 'LineWidth',1.5, 'DisplayName', '原始路径');
plot3(pruned_path(:,1), pruned_path(:,2), pruned_path(:,3), ...
      'k.-', 'LineWidth',2, 'MarkerSize',12, 'DisplayName', '迭代剪枝路径');
if use_bspline
    plot3(final_path(:,1), final_path(:,2), final_path(:,3), ...
          'r-', 'LineWidth',3, 'DisplayName', 'B样条轨迹');
else
    plot3(final_path(:,1), final_path(:,2), final_path(:,3), ...
          'm-', 'LineWidth',3, 'DisplayName', '退回剪枝路径');
end

% legend('Location', 'best');
hold off;

%% ================= 辅助函数 =================

% 自适应最近邻搜索（基于当前点到采样点的能量距离）
function idx = findNearestAdaptive(nodes, sample, goal_z, w_dist, w_climb, adapt_scale)
    n = size(nodes,1);
    min_energy = inf;
    idx = 1;
    for i = 1:n
        dE = energyIncrementAdaptive(nodes(i,:), sample, goal_z, w_dist, w_climb, adapt_scale);
        if dE < min_energy
            min_energy = dE;
            idx = i;
        end
    end
end

% 自适应能量增量：水平距离 + 自适应爬升惩罚
% p1: 当前点, p2: 目标点, goal_z: 目标高度
function inc = energyIncrementAdaptive(p1, p2, goal_z, w_dist, w_climb, adapt_scale)
    dp = p2 - p1;
    xy_len = sqrt(dp(1)^2 + dp(2)^2);
    climb = max(0, dp(3));   % 只计爬升，下降为0
    
    % 计算当前点高度与目标高度的差值
    dz_goal = goal_z - p1(3);
    
    % 自适应爬升系数：当前点低于目标时，减小爬升惩罚；高于目标时，加大爬升惩罚
    if dz_goal > 0
        % p1 低于目标，爬升是有益的，惩罚系数减小
        factor = max(0, 1 - dz_goal / adapt_scale);
    else
        % p1 高于或等于目标，爬升是无益的，惩罚系数增大
        factor = 1 + abs(dz_goal) / adapt_scale;
    end
    climb_weight = w_climb * factor;
    
    inc = w_dist * xy_len + climb_weight * climb;
end

% 线段与AABB碰撞检测
function coll = checkCollision(p1, p2, obstacles)
    coll = false;
    for i = 1:size(obstacles,1)
        bbox = obstacles(i,:);
        inv_dir = 1 ./ (p2 - p1);
        tmin = zeros(1,3); tmax = zeros(1,3);
        for dim = 1:3
            d = p2(dim) - p1(dim);
            if abs(d) < 1e-12
                if p1(dim) < bbox(2*dim-1) || p1(dim) > bbox(2*dim)
                    tmin(dim) = inf; tmax(dim) = -inf;
                else
                    tmin(dim) = -inf; tmax(dim) = inf;
                end
            else
                t1 = (bbox(2*dim-1) - p1(dim)) * inv_dir(dim);
                t2 = (bbox(2*dim)   - p1(dim)) * inv_dir(dim);
                tmin(dim) = min(t1, t2);
                tmax(dim) = max(t1, t2);
            end
        end
        t_enter = max(tmin);
        t_exit  = min(tmax);
        if t_enter <= t_exit && t_exit >= 0 && t_enter <= 1
            coll = true;
            return;
        end
    end
end

% 迭代剪枝
function pruned = iterativePrune(path, obstacles, max_passes)
    pruned = path;
    for pass = 1:max_passes
        old_cnt = size(pruned,1);
        pruned = singlePassPrune(pruned, obstacles);
        if size(pruned,1) == old_cnt
            break;
        end
    end
    fprintf('  剪枝迭代 %d 轮\n', pass);
end

function pruned = singlePassPrune(path, obstacles)
    if size(path,1) < 3
        pruned = path; return;
    end
    pruned = path(1,:);
    cur = 1; total = size(path,1);
    while cur < total
        visible = false;
        for j = total:-1:cur+1
            if ~checkCollision(path(cur,:), path(j,:), obstacles)
                pruned = [pruned; path(j,:)];
                cur = j;
                visible = true;
                break;
            end
        end
        if ~visible
            cur = cur + 1;
            if cur <= total
                pruned = [pruned; path(cur,:)];
            end
        end
    end
end

% B样条
function curve = bsplineSmooth(ctrl_pts, k, n_samples)
    n = size(ctrl_pts,1);
    if n < k, k = n; end
    t = [zeros(1,k-1), linspace(0,1,n-k+2), ones(1,k-1)];
    u = linspace(0,1,n_samples);
    curve = zeros(n_samples,3);
    for i = 1:n_samples
        curve(i,:) = bspline_eval(u(i), k, t, ctrl_pts);
    end
end

function N = bspline_basis(i, k, t, u)
    if k == 1
        if (t(i) <= u && u < t(i+1)) || (u == t(end) && i == length(t)-k)
            N = 1;
        else
            N = 0;
        end
    else
        N = 0;
        if t(i+k-1) - t(i) ~= 0
            N = N + (u - t(i)) / (t(i+k-1)-t(i)) * bspline_basis(i,k-1,t,u);
        end
        if t(i+k) - t(i+1) ~= 0
            N = N + (t(i+k) - u) / (t(i+k)-t(i+1)) * bspline_basis(i+1,k-1,t,u);
        end
    end
end

function pt = bspline_eval(u, k, t, ctrl_pts)
    n = size(ctrl_pts,1);
    pt = [0,0,0];
    for i = 1:n
        N = bspline_basis(i,k,t,u);
        pt = pt + N * ctrl_pts(i,:);
    end
end

% 碰撞检测（点集）
function safe = isPathCollisionFree(path, obstacles)
    safe = true;
    for i = 1:size(path,1)
        if pointInAABB(path(i,:), obstacles)
            safe = false;
            return;
        end
    end
end

function inside = pointInAABB(pt, obstacles)
    inside = false;
    for i = 1:size(obstacles,1)
        b = obstacles(i,:);
        if pt(1) >= b(1) && pt(1) <= b(2) && ...
           pt(2) >= b(3) && pt(2) <= b(4) && ...
           pt(3) >= b(5) && pt(3) <= b(6)
            inside = true;
            return;
        end
    end
end

% 绘制障碍物
function plotAABBObstacles(obstacles)
    for i = 1:size(obstacles,1)
        b = obstacles(i,:);
        x = [b(1), b(2), b(2), b(1), b(1), b(2), b(2), b(1)];
        y = [b(3), b(3), b(4), b(4), b(3), b(3), b(4), b(4)];
        z = [b(5), b(5), b(5), b(5), b(6), b(6), b(6), b(6)];
        faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
        patch('Vertices', [x' y' z'], 'Faces', faces, ...
              'FaceColor', [0.6 0.6 0.6], 'FaceAlpha', 0.3, ...
              'EdgeColor', 'k');
    end
end

function drawInflatedBBox(bbox)
    x = [bbox(1), bbox(2), bbox(2), bbox(1), bbox(1), bbox(2), bbox(2), bbox(1)];
    y = [bbox(3), bbox(3), bbox(4), bbox(4), bbox(3), bbox(3), bbox(4), bbox(4)];
    z = [bbox(5), bbox(5), bbox(5), bbox(5), bbox(6), bbox(6), bbox(6), bbox(6)];
    edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
    for k = 1:size(edges,1)
        plot3([x(edges(k,1)) x(edges(k,2))], ...
              [y(edges(k,1)) y(edges(k,2))], ...
              [z(edges(k,1)) z(edges(k,2))], ...
              'r--', 'LineWidth', 1.2);
    end
end

% 固定权重的能量计算（用于评估）
function E = computeEnergy(path, w_dist, w_climb, w_turn)
    E.dist = 0;
    E.climb = 0;
    E.turn_sum = 0;
    n = size(path,1);
    if n < 2
        E.total = 0;
        return;
    end
    for i = 1:n-1
        dp = path(i+1,:) - path(i,:);
        xy_dist = sqrt(dp(1)^2 + dp(2)^2);
        E.dist = E.dist + xy_dist;
        if dp(3) > 0
            E.climb = E.climb + dp(3);
        end
    end
    for i = 2:n-1
        v1 = path(i,:) - path(i-1,:);
        v2 = path(i+1,:) - path(i,:);
        len1 = norm(v1); len2 = norm(v2);
        if len1 > 1e-9 && len2 > 1e-9
            cos_theta = dot(v1, v2) / (len1 * len2);
            cos_theta = max(min(cos_theta, 1), -1);
            E.turn_sum = E.turn_sum + acos(cos_theta);
        end
    end
    E.total = w_dist * E.dist + w_climb * E.climb + w_turn * E.turn_sum;
end