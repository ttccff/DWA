% 3D RRT vs APF-RRT 对比（自适应能量代价、高度偏置、剪枝、B样条）
clear; close all; clc;

%% ================= 参数设置（全局） =================
% 环境与算法参数
goal_bias        = 0.1;
height_bias_prob = 0.3;
step_size        = 2.0;
max_iter         = 5000;
goal_threshold   = 3.0;
spline_sample    = 200;
inflate_factor   = 1.1;
prune_max_passes = 10;

% 能量代价权重（包括转向权重）
energy_w_dist   = 1.0;
energy_w_climb  = 0.5;
energy_w_turn   = 0.5;      % 转向权重
adapt_scale     = 30.0;

% APF 参数（仅在 APF-RRT 中使用）
K_att           = 0.15;
K_rep           = 2.0;
rho0            = 12.0;
apf_weight      = 0.6;

% 空间边界
x_range = [-20, 120];  y_range = [-20, 120];  z_range = [0, 120];

% 起点与终点
start = [10, 10, 10];
goal  = [90, 90, 40];

% 障碍物（底部全部贴地）
obstacles_original = [
    20, 30, 20, 40,  0, 30;
    50, 65, 40, 60,  0, 40;
    70, 80, 70, 85,  0, 50;
    15, 25, 60, 80,  0, 60;
    75, 90, 15, 30,  0, 55;
    35, 50, 30, 45,  0, 50;
    55, 70, 10, 20,  0, 40;
    5,  15, 65, 75,  0, 35;
    ];

% 膨胀障碍物
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

%% ================= 运行两种算法 =================
fprintf('========== 运行标准 RRT（无 APF） ==========\n');
[path_rrt, nodes_rrt, parent_rrt, stats_rrt] = runRRT(start, goal, obstacles_inflated, ...
    x_range, y_range, z_range, step_size, max_iter, goal_threshold, ...
    goal_bias, height_bias_prob, energy_w_dist, energy_w_climb, energy_w_turn, adapt_scale, ...
    prune_max_passes, spline_sample, false, K_att, K_rep, rho0, apf_weight);

fprintf('\n========== 运行 APF-RRT ==========\n');
[path_apf, nodes_apf, parent_apf, stats_apf] = runRRT(start, goal, obstacles_inflated, ...
    x_range, y_range, z_range, step_size, max_iter, goal_threshold, ...
    goal_bias, height_bias_prob, energy_w_dist, energy_w_climb, energy_w_turn, adapt_scale, ...
    prune_max_passes, spline_sample, true, K_att, K_rep, rho0, apf_weight);

%% ================= 打印对比结果 =================
fprintf('\n========== 性能对比 ==========\n');
fprintf('指标                  RRT         APF-RRT\n');
fprintf('路径长度 (水平)       %.2f        %.2f\n', stats_rrt.E.dist, stats_apf.E.dist);
fprintf('总爬升高度            %.2f        %.2f\n', stats_rrt.E.climb, stats_apf.E.climb);
fprintf('总转向角 (rad)        %.2f        %.2f\n', stats_rrt.E.turn_sum, stats_apf.E.turn_sum);
fprintf('总能耗                %.2f        %.2f\n', stats_rrt.E.total, stats_apf.E.total);
fprintf('迭代次数              %d          %d\n', stats_rrt.iter, stats_apf.iter);
fprintf('路径节点数            %d          %d\n', stats_rrt.num_nodes, stats_apf.num_nodes);
fprintf('是否使用B样条         %s          %s\n', stats_rrt.use_bspline, stats_apf.use_bspline);

%% ================= 可视化对比 =================
figure('Name', 'RRT vs APF-RRT 对比', 'Position', [100,100,1400,600]);

% 子图1：RRT
subplot(1,2,1); hold on; grid on; axis equal;
view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('标准 RRT (无 APF)');
plotEnvironment(obstacles_original, obstacles_inflated);
plot3(start(1),start(2),start(3), 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
% 树
for i = 2:size(nodes_rrt,1)
    p1 = nodes_rrt(parent_rrt(i),:);
    p2 = nodes_rrt(i,:);
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], ...
          'Color', [0.7 0.85 1], 'LineWidth', 0.3);
end
% 路径
plot3(path_rrt(:,1), path_rrt(:,2), path_rrt(:,3), 'b-', 'LineWidth', 2);
% legend('障碍物','膨胀边界','起点','终点','路径','Location','best');

% 子图2：APF-RRT
subplot(1,2,2); hold on; grid on; axis equal;
view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('APF-RRT');
plotEnvironment(obstacles_original, obstacles_inflated);
plot3(start(1),start(2),start(3), 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
for i = 2:size(nodes_apf,1)
    p1 = nodes_apf(parent_apf(i),:);
    p2 = nodes_apf(i,:);
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], ...
          'Color', [0.7 0.85 1], 'LineWidth', 0.3);
end
plot3(path_apf(:,1), path_apf(:,2), path_apf(:,3), 'r-', 'LineWidth', 2);
% legend('障碍物','膨胀边界','起点','终点','路径','Location','best');

sgtitle('标准 RRT vs APF-RRT 路径规划对比');

%% ================= 函数定义 =================

% 主运行函数（增加了 energy_w_turn 参数）
function [final_path, nodes, parent, stats] = runRRT(start, goal, obstacles_inflated, ...
    x_range, y_range, z_range, step_size, max_iter, goal_threshold, ...
    goal_bias, height_bias_prob, energy_w_dist, energy_w_climb, energy_w_turn, adapt_scale, ...
    prune_max_passes, spline_sample, use_apf, K_att, K_rep, rho0, apf_weight)

    % 初始化
    nodes  = start;
    cost   = 0;
    parent = 0;
    goal_reached = false;
    goal_idx = 0;

    for iter = 1:max_iter
        % 采样
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

        % APF 引导
        if use_apf
            f_att = K_att * (goal - sample);
            f_rep = [0, 0, 0];
            for i = 1:size(obstacles_inflated,1)
                [dist_vec, dist] = pointAABBDistance(sample, obstacles_inflated(i,:));
                if dist < rho0 && dist > 0
                    f_rep = f_rep + K_rep * (1/dist - 1/rho0) * (1/dist^2) * (dist_vec / dist);
                end
            end
            total_force = f_att + f_rep;
            if norm(total_force) > 0
                sample_apf = sample + apf_weight * (total_force / norm(total_force));
                sample_apf = max([x_range(1), y_range(1), z_range(1)], ...
                                 min([x_range(2), y_range(2), z_range(2)], sample_apf));
                if ~pointInAABB(sample_apf, obstacles_inflated)
                    sample = sample_apf;
                end
            end
        end

        % 最近节点
        nearest_idx = findNearestAdaptive(nodes, sample, goal(3), energy_w_dist, energy_w_climb, adapt_scale);
        nearest_node = nodes(nearest_idx, :);

        % 扩展
        direction = sample - nearest_node;
        dir_norm  = norm(direction);
        if dir_norm < 1e-6, continue; end
        new_node = nearest_node + (step_size / dir_norm) * direction;

        % 边界检查
        if new_node(1) < x_range(1) || new_node(1) > x_range(2) || ...
           new_node(2) < y_range(1) || new_node(2) > y_range(2) || ...
           new_node(3) < z_range(1) || new_node(3) > z_range(2)
            continue;
        end

        % 碰撞检测
        if checkCollision(nearest_node, new_node, obstacles_inflated)
            continue;
        end

        % 能量增量
        delta_energy = energyIncrementAdaptive(nearest_node, new_node, goal(3), energy_w_dist, energy_w_climb, adapt_scale);
        new_cost = cost(nearest_idx) + delta_energy;

        % 加入树
        nodes  = [nodes; new_node];
        cost   = [cost; new_cost];
        parent = [parent; nearest_idx];
        new_idx = size(nodes, 1);

        % 目标检测
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

    if ~goal_reached
        error('未找到路径！');
    end

    % 提取原始路径
    path_indices = [];
    idx = goal_idx;
    while idx ~= 0
        path_indices = [idx; path_indices];
        idx = parent(idx);
    end
    original_path = nodes(path_indices, :);

    % 剪枝
    pruned_path = iterativePrune(original_path, obstacles_inflated, prune_max_passes);

    % B样条平滑
    smooth_candidate = bsplineSmooth(pruned_path, 4, spline_sample);
    if isPathCollisionFree(smooth_candidate, obstacles_inflated)
        final_path = smooth_candidate;
        use_bspline = true;
    else
        final_path = pruned_path;
        use_bspline = false;
    end

    % 统计（传入 energy_w_turn）
    E = computeEnergy(final_path, energy_w_dist, energy_w_climb, energy_w_turn);
    stats.E = E;
    stats.iter = iter;
    stats.num_nodes = size(nodes,1);
    stats.use_bspline = use_bspline;
end

% ---------- 辅助函数（所有算法共用） ----------
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

function inc = energyIncrementAdaptive(p1, p2, goal_z, w_dist, w_climb, adapt_scale)
    dp = p2 - p1;
    xy_len = sqrt(dp(1)^2 + dp(2)^2);
    climb = max(0, dp(3));
    dz_goal = goal_z - p1(3);
    if dz_goal > 0
        factor = max(0, 1 - dz_goal / adapt_scale);
    else
        factor = 1 + abs(dz_goal) / adapt_scale;
    end
    climb_weight = w_climb * factor;
    inc = w_dist * xy_len + climb_weight * climb;
end

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

function pruned = iterativePrune(path, obstacles, max_passes)
    pruned = path;
    for pass = 1:max_passes
        old_cnt = size(pruned,1);
        pruned = singlePassPrune(pruned, obstacles);
        if size(pruned,1) == old_cnt
            break;
        end
    end
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

function [dist_vec, dist] = pointAABBDistance(pt, bbox)
    nearest = zeros(1,3);
    for dim = 1:3
        if pt(dim) < bbox(2*dim-1)
            nearest(dim) = bbox(2*dim-1);
        elseif pt(dim) > bbox(2*dim)
            nearest(dim) = bbox(2*dim);
        else
            nearest(dim) = pt(dim);
        end
    end
    dist_vec = pt - nearest;
    dist = norm(dist_vec);
    if dist == 0
        dist_vec = [0,0,0];
    end
end

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

% 绘图辅助
function plotEnvironment(obstacles_orig, obstacles_infl)
    % 原始障碍物
    for i = 1:size(obstacles_orig,1)
        b = obstacles_orig(i,:);
        x = [b(1), b(2), b(2), b(1), b(1), b(2), b(2), b(1)];
        y = [b(3), b(3), b(4), b(4), b(3), b(3), b(4), b(4)];
        z = [b(5), b(5), b(5), b(5), b(6), b(6), b(6), b(6)];
        faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
        patch('Vertices', [x' y' z'], 'Faces', faces, ...
              'FaceColor', [0.6 0.6 0.6], 'FaceAlpha', 0.3, ...
              'EdgeColor', 'k');
    end
    % 膨胀边界
    for i = 1:size(obstacles_infl,1)
        bbox = obstacles_infl(i,:);
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
end