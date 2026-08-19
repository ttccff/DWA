clc; clear; close all;

%% ==================== 参数设置 ====================
% 原始参考点（每行一个点，可以是 2D 或 3D）
waypoints_original = [
   0, 0;
1, 0;
1, 1;
-1, 1;
-1, -1;
2, -1;
2, 2;
-2, 2;
-2, -2;
3, -2;
3, 3;
-3, 3;
-3, -3;
4, -3;
4, 4;
-4, 4;
-4, -4;
0, -4
];

% 插值参数：每个原始段之间插入的内部点数（0 表示不插值）
n_interp = 5;

% 是否启用初始时间重新分配（根据路径距离比例分配总时间）
use_time_reallocation = true;

% 原始每段运动时间（总时间 = sum(T_original)）
T_original = ones(1, size(waypoints_original, 1) - 1) * 30;   % 每段 30 秒

% 轨迹多项式阶数
N = 7;

% 中间点连续导数最高阶（速度、加速度、jerk、snap 连续）
cont_order = 4;

% 混合权重（最小 SNAP 与最小 JERK 权重各 0.5）
w_snap = 0.5;
w_jerk = 0.5;

% 转向角时间分配参数
alpha_turn = 0;   % 转向角影响系数（建议 0.3~1.5）
beta_angle = 1.5;   % 角度影响指数（0.5 可削弱大角度影响，1 为线性）

%% ==================== 参考点插值 ====================
m_orig = size(waypoints_original, 1);
waypoints = waypoints_original(1, :);   % 先添加起点

for i = 1:m_orig-1
    p_start = waypoints_original(i, :);
    p_end   = waypoints_original(i+1, :);
    % 在当前原始段内插入 n_interp 个内部点
    for j = 1:n_interp
        alpha = j / (n_interp + 1);
        waypoints = [waypoints; (1-alpha)*p_start + alpha*p_end];
    end
    % 添加当前段的终点（也是下一段的起点）
    waypoints = [waypoints; p_end];
end

%% ==================== 初始时间分配（时间优化前） ====================
if use_time_reallocation
    % 根据相邻航点之间的距离比例重新分配总时间
    diffs = diff(waypoints, 1, 1);
    dists = sqrt(sum(diffs.^2, 2))';   % 行向量
    total_dist = sum(dists);
    if total_dist < 1e-12
        error('总路径距离为零，请检查航点是否重合');
    end
    T_total = sum(T_original);
    T_init = T_total * dists / total_dist;
else
    % 原始段内均匀分配
    if n_interp > 0
        T_init = [];
        for i = 1:length(T_original)
            T_init = [T_init, ...
                repmat(T_original(i) / (n_interp + 1), 1, n_interp + 1)];
        end
    else
        T_init = T_original;
    end
end

% 确保为行向量
T_init = T_init(:)';

%% ==================== 基于转向角的时间重新分配（改进版） ====================
% 保持总时间不变，根据路径转向角调整各段时间
T_opt = allocate_time_by_turning_angle(waypoints, sum(T_init), alpha_turn, beta_angle);

fprintf('初始总时间：%.2f s，转向角分配后总时间：%.2f s\n', sum(T_init), sum(T_opt));

% ---------- 绘制时间分配柱状图（初始 vs 转向角分配） ----------
figure('Name', '时间分配对比');
bar_data = [T_init(:)'; T_opt(:)']';
bar(bar_data, 0.8);
xlabel('段索引');
ylabel('时间 (s)');
title('每段轨迹时间分配对比');
legend('初始时间', '转向角分配时间', 'Location', 'best');
grid on;
text(0.5, max(bar_data(:))*1.1, sprintf('初始总时间: %.2f s, 转向角分配总时间: %.2f s', sum(T_init), sum(T_opt)), ...
    'Units', 'normalized', 'HorizontalAlignment', 'center', 'FontSize', 10);
% ------------------------------------------------

%% ==================== 计算初始时间下的轨迹并绘图 ====================
% 构造二次型矩阵
Q_snap_init  = build_Q_for_derivative(N, T_init, 4);
Q_jerk_init  = build_Q_for_derivative(N, T_init, 3);
Q_mixed_init = w_snap * Q_snap_init + w_jerk * Q_jerk_init;

% 求解多项式系数
coeffs_snap_init  = compute_coeffs(waypoints, N, T_init, cont_order, Q_snap_init);
coeffs_mixed_init = compute_coeffs(waypoints, N, T_init, cont_order, Q_mixed_init);

% 计算轨迹
total_time_init = sum(T_init);
ts_init = linspace(0, total_time_init, 500);
traj_snap_init  = evaluate_trajectory_derivative(coeffs_snap_init,  N, T_init, ts_init, 0);
traj_mixed_init = evaluate_trajectory_derivative(coeffs_mixed_init, N, T_init, ts_init, 0);

% 绘制 Figure 1：初始时间分配的轨迹
figure('Name', '初始时间分配轨迹（最小SNAP 与 SNAP+JERK 混合）');
hold on; grid on; axis equal;
dim = size(waypoints, 2);
if dim >= 3
    view(3);
    plot3(waypoints_original(:,1), waypoints_original(:,2), waypoints_original(:,3), ...
        'k--', 'LineWidth', 1.0, 'DisplayName', '原始折线');
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), ...
        'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r', ...
        'DisplayName', '插值参考点');
    plot3(traj_snap_init(:,1), traj_snap_init(:,2), traj_snap_init(:,3), ...
        'b-', 'LineWidth', 1.5, 'DisplayName', '最小SNAP');
    plot3(traj_mixed_init(:,1), traj_mixed_init(:,2), traj_mixed_init(:,3), ...
        'g--', 'LineWidth', 1.5, 'DisplayName', 'SNAP+JERK 各0.5');
    xlabel('X'); ylabel('Y'); zlabel('Z');
else
    plot(waypoints_original(:,1), waypoints_original(:,2), ...
        'k--', 'LineWidth', 1.0, 'DisplayName', '原始折线');
    plot(waypoints(:,1), waypoints(:,2), ...
        'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r', ...
        'DisplayName', '插值参考点');
    plot(traj_snap_init(:,1), traj_snap_init(:,2), ...
        'b-', 'LineWidth', 1.5, 'DisplayName', '最小SNAP');
    plot(traj_mixed_init(:,1), traj_mixed_init(:,2), ...
        'g--', 'LineWidth', 1.5, 'DisplayName', 'SNAP+JERK 各0.5');
    xlabel('X'); ylabel('Y');
end
legend('Location', 'best');
title('初始时间分配下的轨迹（时间优化前）');
hold off;

%% ==================== 计算转向角分配后时间下的轨迹并绘图 ====================
% 构造二次型矩阵
Q_snap_opt  = build_Q_for_derivative(N, T_opt, 4);
Q_jerk_opt  = build_Q_for_derivative(N, T_opt, 3);
Q_mixed_opt = w_snap * Q_snap_opt + w_jerk * Q_jerk_opt;

% 求解多项式系数
coeffs_snap_opt  = compute_coeffs(waypoints, N, T_opt, cont_order, Q_snap_opt);
coeffs_mixed_opt = compute_coeffs(waypoints, N, T_opt, cont_order, Q_mixed_opt);

% 计算轨迹
total_time_opt = sum(T_opt);
ts_opt = linspace(0, total_time_opt, 500);
traj_snap_opt  = evaluate_trajectory_derivative(coeffs_snap_opt,  N, T_opt, ts_opt, 0);
traj_mixed_opt = evaluate_trajectory_derivative(coeffs_mixed_opt, N, T_opt, ts_opt, 0);

% 绘制 Figure 2：转向角分配后时间分配的轨迹
figure('Name', '转向角时间分配轨迹（最小SNAP 与 SNAP+JERK 混合）');
hold on; grid on; axis equal;
if dim >= 3
    view(3);
    plot3(waypoints_original(:,1), waypoints_original(:,2), waypoints_original(:,3), ...
        'k--', 'LineWidth', 1.0, 'DisplayName', '原始折线');
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), ...
        'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r', ...
        'DisplayName', '插值参考点');
    plot3(traj_snap_opt(:,1), traj_snap_opt(:,2), traj_snap_opt(:,3), ...
        'b-', 'LineWidth', 1.5, 'DisplayName', '最小SNAP');
    plot3(traj_mixed_opt(:,1), traj_mixed_opt(:,2), traj_mixed_opt(:,3), ...
        'g--', 'LineWidth', 1.5, 'DisplayName', 'SNAP+JERK 各0.5');
    xlabel('X'); ylabel('Y'); zlabel('Z');
else
    plot(waypoints_original(:,1), waypoints_original(:,2), ...
        'k--', 'LineWidth', 1.0, 'DisplayName', '原始折线');
    plot(waypoints(:,1), waypoints(:,2), ...
        'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r', ...
        'DisplayName', '插值参考点');
    plot(traj_snap_opt(:,1), traj_snap_opt(:,2), ...
        'b-', 'LineWidth', 1.5, 'DisplayName', '最小SNAP');
    plot(traj_mixed_opt(:,1), traj_mixed_opt(:,2), ...
        'g--', 'LineWidth', 1.5, 'DisplayName', 'SNAP+JERK 各0.5');
    xlabel('X'); ylabel('Y');
end
legend('Location', 'best');
title('转向角时间分配下的轨迹（时间优化后）');
hold off;

%% ==================== 优化后轨迹的每个方向的位置、速度、加速度、jerk 对比 ====================
% 计算转向角分配后两种轨迹的各阶导数
pos_snap   = evaluate_trajectory_derivative(coeffs_snap_opt,  N, T_opt, ts_opt, 0);
vel_snap   = evaluate_trajectory_derivative(coeffs_snap_opt,  N, T_opt, ts_opt, 1);
acc_snap   = evaluate_trajectory_derivative(coeffs_snap_opt,  N, T_opt, ts_opt, 2);
jerk_snap  = evaluate_trajectory_derivative(coeffs_snap_opt,  N, T_opt, ts_opt, 3);

pos_mixed  = evaluate_trajectory_derivative(coeffs_mixed_opt, N, T_opt, ts_opt, 0);
vel_mixed  = evaluate_trajectory_derivative(coeffs_mixed_opt, N, T_opt, ts_opt, 1);
acc_mixed  = evaluate_trajectory_derivative(coeffs_mixed_opt, N, T_opt, ts_opt, 2);
jerk_mixed = evaluate_trajectory_derivative(coeffs_mixed_opt, N, T_opt, ts_opt, 3);

% 参考点对应的时间
wp_times = [0, cumsum(T_opt)];

% 方向名称
direction_names = {'X', 'Y', 'Z'};

for d = 1:dim
    dir_name = direction_names{min(d, 3)};
    figure('Name', sprintf('方向 %s：位置/速度/加速度/Jerk 对比（转向角时间分配后）', dir_name));

    % 位置
    subplot(4,1,1);
    plot(ts_opt, pos_snap(:,d), 'b-', 'LineWidth', 1.5); hold on;
    plot(ts_opt, pos_mixed(:,d), 'g--', 'LineWidth', 1.5);
    plot(wp_times, waypoints(:,d), 'ro', 'MarkerFaceColor', 'r');
    ylabel('位置'); grid on;
    legend('最小SNAP', 'SNAP+JERK混合', '参考点', 'Location', 'best');
    title(sprintf('%s 方向位置对比', dir_name));

    % 速度
    subplot(4,1,2);
    plot(ts_opt, vel_snap(:,d), 'b-', 'LineWidth', 1.5); hold on;
    plot(ts_opt, vel_mixed(:,d), 'g--', 'LineWidth', 1.5);
    ylabel('速度'); grid on;
    legend('最小SNAP', 'SNAP+JERK混合', 'Location', 'best');
    title(sprintf('%s 方向速度对比', dir_name));

    % 加速度
    subplot(4,1,3);
    plot(ts_opt, acc_snap(:,d), 'b-', 'LineWidth', 1.5); hold on;
    plot(ts_opt, acc_mixed(:,d), 'g--', 'LineWidth', 1.5);
    ylabel('加速度'); grid on;
    legend('最小SNAP', 'SNAP+JERK混合', 'Location', 'best');
    title(sprintf('%s 方向加速度对比', dir_name));

    % Jerk
    subplot(4,1,4);
    plot(ts_opt, jerk_snap(:,d), 'b-', 'LineWidth', 1.5); hold on;
    plot(ts_opt, jerk_mixed(:,d), 'g--', 'LineWidth', 1.5);
    ylabel('Jerk'); xlabel('时间'); grid on;
    legend('最小SNAP', 'SNAP+JERK混合', 'Location', 'best');
    title(sprintf('%s 方向Jerk对比', dir_name));
end

%% ==================== 局部函数 ====================

% 基于转向角的时间分配函数（改进版）
function T = allocate_time_by_turning_angle(waypoints, total_time, alpha, beta)
% 根据路径点转向角分配每段时间
% 输入：
%   waypoints : N x dim 航点矩阵
%   total_time: 总时间标量
%   alpha     : 转向角影响系数（非负，0 表示仅按距离分配）
%   beta      : 角度影响指数（0.5 可削弱大角度影响，1 为线性）
% 输出：
%   T         : 1 x (N-1) 行向量，每段分配的时间

    N = size(waypoints, 1);
    K = N - 1;               % 段数

    % 1. 计算每段距离
    dists = zeros(1, K);
    for i = 1:K
        dists(i) = norm(waypoints(i+1, :) - waypoints(i, :));
    end

    % 2. 计算每个航点的转向角（弧度）
    angles = zeros(1, N);    % 航点转向角
    for i = 2:N-1
        v1 = waypoints(i, :) - waypoints(i-1, :);
        v2 = waypoints(i+1, :) - waypoints(i, :);
        v1_norm = norm(v1);
        v2_norm = norm(v2);
        if v1_norm > 1e-9 && v2_norm > 1e-9
            cos_theta = dot(v1, v2) / (v1_norm * v2_norm);
            cos_theta = max(-1, min(1, cos_theta));  % 防止数值误差
            angles(i) = acos(cos_theta);
        else
            angles(i) = 0;   % 若长度为零，忽略
        end
    end

    % 3. 为每段计算权重
    weights = zeros(1, K);
    for i = 1:K
        % 段两端的转向角平均值
        theta_avg = (angles(i) + angles(i+1)) / 2;
        % 归一化到 [0, 1]
        theta_norm = theta_avg / pi;
        % 权重 = 距离 * (1 + alpha * theta_norm^beta)
        weights(i) = dists(i) * (1 + alpha * theta_norm^beta);
    end

    % 4. 归一化到总时间
    if sum(weights) < 1e-12
        error('权重总和为零，请检查航点是否重合');
    end
    T = total_time * weights / sum(weights);
end

% 构造某个导数阶数对应的二次型矩阵
% derivative_order = 3 表示 jerk，= 4 表示 snap
function Q = build_Q_for_derivative(N, T_segments, derivative_order)
    K = length(T_segments);
    n = K * (N + 1);
    Q = zeros(n, n);

    for seg = 1:K
        T = T_segments(seg);
        Qs = zeros(N + 1, N + 1);

        for k = derivative_order:N
            ck = factorial(k) / factorial(k - derivative_order);
            for l = derivative_order:N
                cl = factorial(l) / factorial(l - derivative_order);
                Qs(k + 1, l + 1) = ck * cl * ...
                    T^(k + l - 2 * derivative_order + 1) / ...
                    (k + l - 2 * derivative_order + 1);
            end
        end

        idx = (seg - 1) * (N + 1) + 1 : seg * (N + 1);
        Q(idx, idx) = Qs;
    end
end

% 构造等式约束 Aeq * coeff = beq
% wp: 某一维度的参考点列向量
function [Aeq, beq] = build_constraints(wp, N, T_segments, cont_order)
    m = length(wp);      % 参考点数量
    K = m - 1;           % 轨迹段数
    n = K * (N + 1);

    % 约束行数
    pos_rows   = 2 * m - 2;
    cont_rows  = cont_order * (m - 2);
    start_rows = cont_order;
    end_rows   = cont_order;
    total_rows = pos_rows + cont_rows + start_rows + end_rows;

    Aeq = zeros(total_rows, n);
    beq = zeros(total_rows, 1);

    row_idx = 0;

    % ---- 起点位置 ----
    row_idx = row_idx + 1;
    Aeq(row_idx, 1:(N + 1)) = derivative_row(N, 0, 0);
    beq(row_idx) = wp(1);

    % ---- 中间航点位置 ----
    for j = 2:m - 1
        prev_seg = j - 1;
        next_seg = j;
        T_prev = T_segments(prev_seg);

        idx_prev = (prev_seg - 1) * (N + 1) + 1 : prev_seg * (N + 1);
        idx_next = (next_seg - 1) * (N + 1) + 1 : next_seg * (N + 1);

        % 前一段末端到达该参考点
        row_idx = row_idx + 1;
        Aeq(row_idx, idx_prev) = derivative_row(N, 0, T_prev);
        beq(row_idx) = wp(j);

        % 后一段起点也从该参考点出发
        row_idx = row_idx + 1;
        Aeq(row_idx, idx_next) = derivative_row(N, 0, 0);
        beq(row_idx) = wp(j);
    end

    % ---- 终点位置 ----
    row_idx = row_idx + 1;
    idx_last = (K - 1) * (N + 1) + 1 : K * (N + 1);
    Aeq(row_idx, idx_last) = derivative_row(N, 0, T_segments(K));
    beq(row_idx) = wp(m);

    % ---- 中间点导数连续 ----
    for j = 2:m - 1
        prev_seg = j - 1;
        next_seg = j;
        T_prev = T_segments(prev_seg);

        idx_prev = (prev_seg - 1) * (N + 1) + 1 : prev_seg * (N + 1);
        idx_next = (next_seg - 1) * (N + 1) + 1 : next_seg * (N + 1);

        for r = 1:cont_order
            row_idx = row_idx + 1;
            Aeq(row_idx, idx_prev) = derivative_row(N, r, T_prev);
            Aeq(row_idx, idx_next) = -derivative_row(N, r, 0);
            beq(row_idx) = 0;
        end
    end

    % ---- 起点高阶导数为 0 ----
    for r = 1:cont_order
        row_idx = row_idx + 1;
        Aeq(row_idx, 1:(N + 1)) = derivative_row(N, r, 0);
        beq(row_idx) = 0;
    end

    % ---- 终点高阶导数为 0 ----
    for r = 1:cont_order
        row_idx = row_idx + 1;
        Aeq(row_idx, idx_last) = derivative_row(N, r, T_segments(K));
        beq(row_idx) = 0;
    end
end

% 求多项式第 r 阶导数在时间 t 处的系数行向量
% 例如 r = 0 表示位置，r = 1 表示速度，r = 2 表示加速度
function row = derivative_row(N, r, t)
    row = zeros(1, N + 1);
    if r > N
        return;
    end

    for k = r:N
        row(k + 1) = factorial(k) / factorial(k - r) * t^(k - r);
    end
end

% 求解等式约束二次规划：
% min  0.5 * x' * Q * x
% s.t. Aeq * x = beq
function coeffs = solve_eq_qp(Q, Aeq, beq)
    n = size(Q, 1);
    m = size(Aeq, 1);

    % KKT 系统
    KKT = [Q, Aeq'; Aeq, zeros(m, m)];
    rhs = [zeros(n, 1); beq];

    % 如果 KKT 矩阵接近奇异，加入小正则化，提高数值稳定性
    if rcond(KKT) < 1e-12
        warning('KKT 矩阵接近奇异，加入小正则化');
        KKT = KKT + 1e-8 * eye(size(KKT));
    end

    sol = KKT \ rhs;
    coeffs = sol(1:n);
end

% 对每个维度分别求解轨迹系数
function coeffs = compute_coeffs(waypoints, N, T_segments, cont_order, Q)
    dim = size(waypoints, 2);
    K = length(T_segments);

    if size(waypoints, 1) - 1 ~= K
        error('航点数量与段时间数量不匹配');
    end

    n = K * (N + 1);
    coeffs = zeros(n, dim);

    for d = 1:dim
        wp = waypoints(:, d);
        [Aeq, beq] = build_constraints(wp, N, T_segments, cont_order);
        coeffs(:, d) = solve_eq_qp(Q, Aeq, beq);
    end
end

% 在采样时间 ts 上计算整条轨迹的指定导数阶数
% deriv_order = 0 位置，1 速度，2 加速度，3 jerk，4 snap
function traj_der = evaluate_trajectory_derivative(coeffs, N, T_segments, ts, deriv_order)
    K = length(T_segments);
    if K == 1
        seg_starts = 0;
    else
        seg_starts = [0, cumsum(T_segments(1:end - 1))];
    end

    dim = size(coeffs, 2);
    traj_der = zeros(length(ts), dim);

    for i = 1:length(ts)
        t = ts(i);

        seg = find(t >= seg_starts, 1, 'last');
        if isempty(seg)
            seg = 1;
        end
        if seg > K
            seg = K;
        end

        tau = t - seg_starts(seg);
        tau = max(0, min(tau, T_segments(seg)));

        idx = (seg - 1) * (N + 1) + 1 : seg * (N + 1);
        coeffs_seg = coeffs(idx, :);
        traj_der(i, :) = eval_poly_derivative(coeffs_seg, N, tau, deriv_order);
    end
end

% 计算单段多项式在给定局部时间 tau 处的指定导数
function val = eval_poly_derivative(coeffs_seg, N, tau, deriv_order)
    val = zeros(1, size(coeffs_seg, 2));
    if deriv_order > N
        return;
    end

    for k = deriv_order:N
        coeff_factor = factorial(k) / factorial(k - deriv_order);
        val = val + coeffs_seg(k + 1, :) * coeff_factor * tau^(k - deriv_order);
    end
end