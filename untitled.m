% 3D RRT vs APF-RRT 对比（APF仅在XY平面作用，无Z方向势场）
clear; close all; clc;
rng('shuffle');

%% ================= 参数设置 =================
goal_bias        = 0;
height_bias_prob = 0;      % 高度偏置采样（已禁用）
step_size        = 5.0;    % 基础步长
max_iter         = 5000;
goal_threshold   = 3.0;
inflate_factor   = 1.1;

% 自适应步长参数（仅APF-RRT）
empty_scale      = 1.2;    % 空旷区域步长放大系数
min_scale        = 1;    % 障碍物附近最小步长比例

% APF 基础参数（仅用于XY平面）
K_att_base       = 0.3;    % 基础引力增益
K_rep            = 2.0;
rho0             = 20.0;
apf_weight       = 1.0;

% 空间边界
x_range = [-20, 100];  y_range = [-20, 100];  z_range = [0, 60];
start = [10, 10, 10];
goal  = [90, 90, 20];

% ================= 障碍物定义（全部底部贴地，z1=0） =================
obstacles_original = [
    20, 30, 20, 40,  0, 30;
    50, 65, 40, 60,  0, 40;
    70, 80, 70, 85,  0, 50;
    15, 25, 60, 80,  0, 60;
    75, 90, 15, 30,  0, 55;
    35, 50, 30, 45,  0, 50;
    55, 70, 10, 20,  0, 40;
    5,  15, 65, 75,  0, 35;
    85, 100, 50, 65,  0, 45;
    30, 45,  70, 85,  0, 50;
    60, 75,  80, 95,  0, 55;
    -10, 5,  30, 45,  0, 40;
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

%% ================= 多次仿真统计 =================
num_trials = 20;
results_rrt = struct('dist', [], 'climb', [], 'turn', [], 'iter', [], 'nodes', [], 'path_nodes', []);
results_apf = struct('dist', [], 'climb', [], 'turn', [], 'iter', [], 'nodes', [], 'path_nodes', []);
success_rrt = false(1, num_trials);
success_apf = false(1, num_trials);

last_path_rrt = []; last_nodes_rrt = []; last_parent_rrt = [];
last_path_apf = []; last_nodes_apf = []; last_parent_apf = [];

fprintf('开始运行 %d 次仿真（APF仅在XY平面作用）...\n', num_trials);

for trial = 1:num_trials
    fprintf('第 %d / %d 次：', trial, num_trials);
    
    try
        [path, nodes, parent, stats, ok] = runRRT(start, goal, obstacles_inflated, ...
            x_range, y_range, z_range, step_size, max_iter, goal_threshold, ...
            goal_bias, height_bias_prob, ...
            false, K_att_base, K_rep, rho0, apf_weight, empty_scale, min_scale);
        if ok
            results_rrt.dist(end+1)   = stats.dist;
            results_rrt.climb(end+1)  = stats.climb;
            results_rrt.turn(end+1)   = stats.turn;
            results_rrt.iter(end+1)   = stats.iter;
            results_rrt.nodes(end+1)  = stats.nodes;
            results_rrt.path_nodes(end+1) = stats.path_nodes;
            success_rrt(trial) = true;
            last_path_rrt = path; last_nodes_rrt = nodes; last_parent_rrt = parent;
        end
    catch
    end

    try
        [path, nodes, parent, stats, ok] = runRRT(start, goal, obstacles_inflated, ...
            x_range, y_range, z_range, step_size, max_iter, goal_threshold, ...
            goal_bias, height_bias_prob, ...
            true, K_att_base, K_rep, rho0, apf_weight, empty_scale, min_scale);
        if ok
            results_apf.dist(end+1)   = stats.dist;
            results_apf.climb(end+1)  = stats.climb;
            results_apf.turn(end+1)   = stats.turn;
            results_apf.iter(end+1)   = stats.iter;
            results_apf.nodes(end+1)  = stats.nodes;
            results_apf.path_nodes(end+1) = stats.path_nodes;
            success_apf(trial) = true;
            last_path_apf = path; last_nodes_apf = nodes; last_parent_apf = parent;
        end
    catch
    end
    
    fprintf(' RRT: %s, APF-RRT: %s\n', ...
        iff(success_rrt(trial), '成功', '失败'), ...
        iff(success_apf(trial), '成功', '失败'));
end

%% ================= 统计结果输出 =================
fprintf('\n========== 统计对比（基于 %d 次仿真） ==========\n', num_trials);
fprintf('标准 RRT 成功率：%d / %d (%.1f%%)\n', sum(success_rrt), num_trials, 100*sum(success_rrt)/num_trials);
fprintf('APF-RRT（XY平面）成功率：%d / %d (%.1f%%)\n', sum(success_apf), num_trials, 100*sum(success_apf)/num_trials);

rrt_dist  = results_rrt.dist;  apf_dist  = results_apf.dist;
rrt_climb = results_rrt.climb; apf_climb = results_apf.climb;
rrt_turn  = results_rrt.turn;  apf_turn  = results_apf.turn;
rrt_iter  = results_rrt.iter;  apf_iter  = results_apf.iter;
rrt_nodes = results_rrt.nodes; apf_nodes = results_apf.nodes;
rrt_path_nodes = results_rrt.path_nodes; apf_path_nodes = results_apf.path_nodes;

fprintf('\n指标                RRT (均值±标准差)          APF-RRT (均值±标准差)\n');
fprintf('路径长度 (水平)     %.2f ± %.2f            %.2f ± %.2f\n', ...
    mean(rrt_dist), std(rrt_dist), mean(apf_dist), std(apf_dist));
fprintf('总爬升高度          %.2f ± %.2f            %.2f ± %.2f\n', ...
    mean(rrt_climb), std(rrt_climb), mean(apf_climb), std(apf_climb));
fprintf('总转向角 (rad)      %.2f ± %.2f            %.2f ± %.2f\n', ...
    mean(rrt_turn), std(rrt_turn), mean(apf_turn), std(apf_turn));
fprintf('迭代次数            %.0f ± %.0f            %.0f ± %.0f\n', ...
    mean(rrt_iter), std(rrt_iter), mean(apf_iter), std(apf_iter));
fprintf('搜索节点数          %.0f ± %.0f            %.0f ± %.0f\n', ...
    mean(rrt_nodes), std(rrt_nodes), mean(apf_nodes), std(apf_nodes));
fprintf('路径节点数          %.0f ± %.0f            %.0f ± %.0f\n', ...
    mean(rrt_path_nodes), std(rrt_path_nodes), mean(apf_path_nodes), std(apf_path_nodes));

%% ================= 图1：三维路径示例 =================
if isempty(last_path_rrt) || isempty(last_path_apf)
    rng(42);
    [last_path_rrt, last_nodes_rrt, last_parent_rrt, ~, ~] = runRRT(start, goal, obstacles_inflated, ...
        x_range, y_range, z_range, step_size, max_iter, goal_threshold, ...
        goal_bias, height_bias_prob, ...
        false, K_att_base, K_rep, rho0, apf_weight, empty_scale, min_scale);
    [last_path_apf, last_nodes_apf, last_parent_apf, ~, ~] = runRRT(start, goal, obstacles_inflated, ...
        x_range, y_range, z_range, step_size, max_iter, goal_threshold, ...
        goal_bias, height_bias_prob, ...
        true, K_att_base, K_rep, rho0, apf_weight, empty_scale, min_scale);
end

figure('Name', '路径示例对比（APF仅XY平面）', 'Position', [100, 100, 800, 600]);
hold on; grid on; axis equal; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title(sprintf('RRT vs APF-RRT (APF仅XY平面) 成功率 RRT:%.1f%% APF:%.1f%%', ...
    100*sum(success_rrt)/num_trials, 100*sum(success_apf)/num_trials));
plotEnvironment(obstacles_original, obstacles_inflated);
plot3(start(1),start(2),start(3), 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');

if ~isempty(last_nodes_rrt)
    for i = 2:size(last_nodes_rrt,1)
        p1 = last_nodes_rrt(last_parent_rrt(i),:);
        p2 = last_nodes_rrt(i,:);
        plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], ...
              'Color', [0.7 0.85 1], 'LineWidth', 0.3);
    end
end
if ~isempty(last_path_rrt)
    plot3(last_path_rrt(:,1), last_path_rrt(:,2), last_path_rrt(:,3), 'b-', 'LineWidth', 2);
end

if ~isempty(last_nodes_apf)
    for i = 2:size(last_nodes_apf,1)
        p1 = last_nodes_apf(last_parent_apf(i),:);
        p2 = last_nodes_apf(i,:);
        plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], ...
              'Color', [1 0.7 0.7], 'LineWidth', 0.3);
    end
end
if ~isempty(last_path_apf)
    plot3(last_path_apf(:,1), last_path_apf(:,2), last_path_apf(:,3), 'r-', 'LineWidth', 2);
end

legend('障碍物','膨胀边界','起点','终点','RRT树','RRT路径','APF树','APF路径','Location','best');
axis([x_range, y_range, z_range]);
hold off;

%% ================= 图2：六个指标独立子图 =================
metrics = {'水平长度', '爬升高度', '转向角 (rad)', '迭代次数', '搜索节点数', '路径节点数'};
rrt_means = [mean(rrt_dist), mean(rrt_climb), mean(rrt_turn), mean(rrt_iter), mean(rrt_nodes), mean(rrt_path_nodes)];
apf_means = [mean(apf_dist), mean(apf_climb), mean(apf_turn), mean(apf_iter), mean(apf_nodes), mean(apf_path_nodes)];
rrt_stds  = [std(rrt_dist), std(rrt_climb), std(rrt_turn), std(rrt_iter), std(rrt_nodes), std(rrt_path_nodes)];
apf_stds  = [std(apf_dist), std(apf_climb), std(apf_turn), std(apf_iter), std(apf_nodes), std(apf_path_nodes)];

figure('Name', '统计指标独立对比（APF仅XY平面）', 'Position', [200, 200, 1200, 800]);
for i = 1:6
    subplot(2,3,i);
    bar_data = [rrt_means(i), apf_means(i)];
    err_data = [rrt_stds(i), apf_stds(i)];
    b1 = bar(1, bar_data(1), 'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'k');
    hold on;
    b2 = bar(2, bar_data(2), 'FaceColor', [0.9 0.3 0.3], 'EdgeColor', 'k');
    errorbar(1:2, bar_data, err_data, 'k.', 'LineWidth', 2, 'MarkerSize', 20);
    set(gca, 'XTick', 1:2, 'XTickLabel', {'RRT', 'APF-RRT'});
    ylabel(metrics{i});
    title(metrics{i});
    grid on;
    hold off;
end
sgtitle(sprintf('40次独立仿真（APF仅XY平面）– 各指标均值与标准差 (成功率 RRT:%.1f%% APF:%.1f%%)', ...
    100*sum(success_rrt)/num_trials, 100*sum(success_apf)/num_trials));

%% ================= 辅助函数 =================
function [final_path, nodes, parent, stats, success] = runRRT(start, goal, obstacles_inflated, ...
    x_range, y_range, z_range, step_size, max_iter, goal_threshold, ...
    goal_bias, height_bias_prob, ...
    use_apf, K_att_base, K_rep, rho0, apf_weight, empty_scale, min_scale)

    nodes  = start;
    parent = 0;
    cost   = 0;
    goal_reached = false;
    goal_idx = 0;

    for iter = 1:max_iter
        % ---- 采样 ----
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

        % ---- 最近邻 ----
        dists = vecnorm(nodes - sample, 2, 2);
        [~, nearest_idx] = min(dists);
        nearest_node = nodes(nearest_idx, :);

        % ---- APF 引导（仅在XY平面） ----
        if use_apf
            % 引力：仅保留XY分量
            f_att = K_att_base * (goal - sample);
            f_att(3) = 0;   % Z方向引力置零

            % 斥力：仅保留XY分量
            f_rep = [0,0,0];
            for i = 1:size(obstacles_inflated,1)
                [dist_vec, dist] = pointAABBDistance(sample, obstacles_inflated(i,:));
                if dist < rho0 && dist > 0
                    f_rep_i = K_rep * (1/dist - 1/rho0) * (1/dist^2) * (dist_vec / dist);
                    f_rep_i(3) = 0;   % Z方向斥力置零
                    f_rep = f_rep + f_rep_i;
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

        % ---- 自适应步长（仅APF） ----
        if use_apf
            d_min = inf;
            for i = 1:size(obstacles_inflated,1)
                [~, dist] = pointAABBDistance(nearest_node, obstacles_inflated(i,:));
                if dist < d_min, d_min = dist; end
            end
            if d_min < rho0
                ratio = d_min / rho0;
                adaptive_step = step_size * (min_scale + (1 - min_scale) * ratio);
            else
                adaptive_step = step_size * empty_scale;
            end
        else
            adaptive_step = step_size;
        end

        % ---- 扩展 ----
        direction = sample - nearest_node;
        dir_norm  = norm(direction);
        if dir_norm < 1e-6, continue; end
        new_node = nearest_node + (adaptive_step / dir_norm) * direction;

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

        % 计算新节点代价
        new_cost = cost(nearest_idx) + norm(new_node - nearest_node);

        % ---- 加入树 ----
        nodes  = [nodes; new_node];
        parent = [parent; nearest_idx];
        cost   = [cost; new_cost];
        new_idx = size(nodes, 1);

        % ---- 目标检测 ----
        if ~goal_reached && norm(new_node - goal) <= goal_threshold
            if ~checkCollision(new_node, goal, obstacles_inflated)
                goal_cost = cost(new_idx) + norm(goal - new_node);
                nodes  = [nodes; goal];
                parent = [parent; new_idx];
                cost   = [cost; goal_cost];
                goal_idx = size(nodes, 1);
                goal_reached = true;
                break;
            end
        end
    end

    if ~goal_reached
        final_path = []; nodes = []; parent = []; 
        stats = struct('dist', NaN, 'climb', NaN, 'turn', NaN, 'iter', NaN, 'nodes', NaN, 'path_nodes', NaN);
        success = false;
        return;
    end

    % ---- 提取路径 ----
    path_indices = []; idx = goal_idx;
    while idx ~= 0
        path_indices = [idx; path_indices];
        idx = parent(idx);
    end
    original_path = nodes(path_indices, :);
    final_path = original_path;   % 无平滑

    % ---- 统计指标 ----
    stats = computeRawStats(final_path);
    stats.iter = iter;
    stats.nodes = size(nodes,1);           % 搜索节点数
    stats.path_nodes = size(final_path,1); % 最终路径节点数
    success = true;
end

% ---- 碰撞检测（线段与AABB） ----
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
        t_enter = max(tmin); t_exit = min(tmax);
        if t_enter <= t_exit && t_exit >= 0 && t_enter <= 1
            coll = true; return;
        end
    end
end

% ---- 点是否在障碍物内 ----
function inside = pointInAABB(pt, obstacles)
    inside = false;
    for i = 1:size(obstacles,1)
        b = obstacles(i,:);
        if pt(1) >= b(1) && pt(1) <= b(2) && ...
           pt(2) >= b(3) && pt(2) <= b(4) && ...
           pt(3) >= b(5) && pt(3) <= b(6)
            inside = true; return;
        end
    end
end

% ---- 点到AABB的距离 ----
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
    if dist == 0, dist_vec = [0,0,0]; end
end

% ---- 计算原始统计指标（不含节点数） ----
function stats = computeRawStats(path)
    stats.dist = 0; stats.climb = 0; stats.turn = 0;
    n = size(path,1);
    if n < 2, return; end
    for i = 1:n-1
        dp = path(i+1,:) - path(i,:);
        xy_dist = sqrt(dp(1)^2 + dp(2)^2);
        stats.dist = stats.dist + xy_dist;
        if dp(3) > 0
            stats.climb = stats.climb + dp(3);
        end
    end
    for i = 2:n-1
        v1 = path(i,:) - path(i-1,:);
        v2 = path(i+1,:) - path(i,:);
        len1 = norm(v1); len2 = norm(v2);
        if len1 > 1e-9 && len2 > 1e-9
            cos_theta = dot(v1, v2) / (len1 * len2);
            cos_theta = max(min(cos_theta, 1), -1);
            stats.turn = stats.turn + acos(cos_theta);
        end
    end
end

% ---- 绘制环境 ----
function plotEnvironment(obstacles_orig, obstacles_infl)
    for i = 1:size(obstacles_orig,1)
        b = obstacles_orig(i,:);
        x = [b(1), b(2), b(2), b(1), b(1), b(2), b(2), b(1)];
        y = [b(3), b(3), b(4), b(4), b(3), b(3), b(4), b(4)];
        z = [b(5), b(5), b(5), b(5), b(6), b(6), b(6), b(6)];
        faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
        patch('Vertices', [x' y' z'], 'Faces', faces, ...
              'FaceColor', [0.6 0.6 0.6], 'FaceAlpha', 0.3, 'EdgeColor', 'k');
    end
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

% ---- 辅助条件判断 ----
function s = iff(cond, tstr, fstr)
    if cond, s = tstr; else, s = fstr; end
end