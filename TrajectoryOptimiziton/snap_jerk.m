clc; clear; close all;

%% ==================== 参数设置 ====================
% 原始参考点（用户定义）
waypoints_original = [
    0, 0, 0;
    1, 4, 0;
    3, 8, 0;
    6, 8, 0;
    6, 4, 0;
    6, 0, 0
];

% 插值参数：每个原始段之间插入的点数（0 表示不插值）
n_interp = 5;   % 例如 5 表示每段插入 5 个点，加上端点共 6 个子段

%% ==================== 参考点插值 ====================
if n_interp > 0
    % 原始航点数
    m_orig = size(waypoints_original, 1);
    % 新的密集航点
    waypoints = [];
    for i = 1:m_orig-1
        p_start = waypoints_original(i, :);
        p_end   = waypoints_original(i+1, :);
        % 在这一段内生成 n_interp 个内部点（不包括起点，包括终点）
        for j = 0:n_interp
            alpha = j / (n_interp + 1);
            waypoints = [waypoints; (1-alpha)*p_start + alpha*p_end];
        end
        % 注意：终点会在下一次循环作为起点重复，因此需要去重
    end
    % 上面会在每段末尾添加终点，下一段开头也添加相同点，产生重复
    % 修正：使用 unique 或直接重新构造
    % 更简洁的方法：
    waypoints = [];
    for i = 1:m_orig-1
        p_start = waypoints_original(i, :);
        p_end   = waypoints_original(i+1, :);
        % 只添加内部点和终点，不添加起点（第一段除外）
        if i == 1
            waypoints = [waypoints; p_start];
        end
        for j = 1:n_interp
            alpha = j / (n_interp + 1);
            waypoints = [waypoints; (1-alpha)*p_start + alpha*p_end];
        end
        waypoints = [waypoints; p_end];
    end
else
    waypoints = waypoints_original;
end

% 每段原始时间（用户可修改）
T_original = ones(1, size(waypoints_original, 1) - 1) * 30;  % 每段 30 秒
if n_interp > 0
    % 总时间不变，均分给每个子段
    T_segments = [];
    for i = 1:length(T_original)
        T_segments = [T_segments, ...
            repmat(T_original(i) / (n_interp + 1), 1, n_interp + 1)];
    end
else
    T_segments = T_original;
end

N = 7;               % 多项式阶数
cont_order = 4;      % 中间点连续导数最高阶：速度、加速度、jerk、snap 连续

% 混合权重
w_snap = 0.5;
w_jerk = 0.5;

%% ==================== 构造二次型矩阵 ====================
Q_snap  = build_Q_for_derivative(N, T_segments, 4);
Q_jerk  = build_Q_for_derivative(N, T_segments, 3);
Q_mixed = w_snap * Q_snap + w_jerk * Q_jerk;

%% ==================== 求解多项式系数 ====================
coeffs_snap  = compute_coeffs(waypoints, N, T_segments, cont_order, Q_snap);
coeffs_mixed = compute_coeffs(waypoints, N, T_segments, cont_order, Q_mixed);

%% ==================== 计算轨迹 ====================
total_time = sum(T_segments);
ts = linspace(0, total_time, 500);

traj_snap  = evaluate_trajectory_derivative(coeffs_snap,  N, T_segments, ts, 0);
traj_mixed = evaluate_trajectory_derivative(coeffs_mixed, N, T_segments, ts, 0);

%% ==================== 3D 轨迹对比图 ====================
dim = size(waypoints, 2);

figure('Name', '最小SNAP 与 SNAP+JERK 混合平滑轨迹（含插值参考点）');
hold on; grid on; axis equal;

if dim >= 3
    view(3);
    % 绘制原始航点折线（虚线）
    plot3(waypoints_original(:,1), waypoints_original(:,2), waypoints_original(:,3), ...
        'k--', 'LineWidth', 1.0, 'DisplayName', '原始折线');
    % 绘制密集参考点
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), ...
        'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r', ...
        'DisplayName', '插值参考点');
    % 绘制两条轨迹
    plot3(traj_snap(:,1), traj_snap(:,2), traj_snap(:,3), ...
        'b-', 'LineWidth', 1.5, 'DisplayName', '最小SNAP');
    plot3(traj_mixed(:,1), traj_mixed(:,2), traj_mixed(:,3), ...
        'g--', 'LineWidth', 1.5, 'DisplayName', 'SNAP+JERK 各0.5');
    xlabel('X'); ylabel('Y'); zlabel('Z');
else
    plot(waypoints_original(:,1), waypoints_original(:,2), ...
        'k--', 'LineWidth', 1.0, 'DisplayName', '原始折线');
    plot(waypoints(:,1), waypoints(:,2), ...
        'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r', ...
        'DisplayName', '插值参考点');
    plot(traj_snap(:,1), traj_snap(:,2), ...
        'b-', 'LineWidth', 1.5, 'DisplayName', '最小SNAP');
    plot(traj_mixed(:,1), traj_mixed(:,2), ...
        'g--', 'LineWidth', 1.5, 'DisplayName', 'SNAP+JERK 各0.5');
    xlabel('X'); ylabel('Y');
end

legend('Location', 'best');
title('参考点平滑连接对比（含插值）');
hold off;

%% ==================== 每个方向的位置、速度、加速度、jerk 对比 ====================
% 计算两种轨迹的各阶导数
pos_snap   = evaluate_trajectory_derivative(coeffs_snap,  N, T_segments, ts, 0);
vel_snap   = evaluate_trajectory_derivative(coeffs_snap,  N, T_segments, ts, 1);
acc_snap   = evaluate_trajectory_derivative(coeffs_snap,  N, T_segments, ts, 2);
jerk_snap  = evaluate_trajectory_derivative(coeffs_snap,  N, T_segments, ts, 3);

pos_mixed  = evaluate_trajectory_derivative(coeffs_mixed, N, T_segments, ts, 0);
vel_mixed  = evaluate_trajectory_derivative(coeffs_mixed, N, T_segments, ts, 1);
acc_mixed  = evaluate_trajectory_derivative(coeffs_mixed, N, T_segments, ts, 2);
jerk_mixed = evaluate_trajectory_derivative(coeffs_mixed, N, T_segments, ts, 3);

% 参考点对应的时间
wp_times = [0, cumsum(T_segments)];

% 方向名称
direction_names = {'X', 'Y', 'Z'};

for d = 1:dim
    dir_name = direction_names{min(d, 3)};
    figure('Name', sprintf('方向 %s：位置/速度/加速度/Jerk 对比', dir_name));

    % 位置
    subplot(4,1,1);
    plot(ts, pos_snap(:,d), 'b-', 'LineWidth', 1.5); hold on;
    plot(ts, pos_mixed(:,d), 'g--', 'LineWidth', 1.5);
    plot(wp_times, waypoints(:,d), 'ro', 'MarkerFaceColor', 'r');
    ylabel('位置'); grid on;
    legend('最小SNAP', 'SNAP+JERK混合', '参考点', 'Location', 'best');
    title(sprintf('%s 方向位置对比', dir_name));

    % 速度
    subplot(4,1,2);
    plot(ts, vel_snap(:,d), 'b-', 'LineWidth', 1.5); hold on;
    plot(ts, vel_mixed(:,d), 'g--', 'LineWidth', 1.5);
    ylabel('速度'); grid on;
    legend('最小SNAP', 'SNAP+JERK混合', 'Location', 'best');
    title(sprintf('%s 方向速度对比', dir_name));

    % 加速度
    subplot(4,1,3);
    plot(ts, acc_snap(:,d), 'b-', 'LineWidth', 1.5); hold on;
    plot(ts, acc_mixed(:,d), 'g--', 'LineWidth', 1.5);
    ylabel('加速度'); grid on;
    legend('最小SNAP', 'SNAP+JERK混合', 'Location', 'best');
    title(sprintf('%s 方向加速度对比', dir_name));

    % Jerk
    subplot(4,1,4);
    plot(ts, jerk_snap(:,d), 'b-', 'LineWidth', 1.5); hold on;
    plot(ts, jerk_mixed(:,d), 'g--', 'LineWidth', 1.5);
    ylabel('Jerk'); xlabel('时间'); grid on;
    legend('最小SNAP', 'SNAP+JERK混合', 'Location', 'best');
    title(sprintf('%s 方向Jerk对比', dir_name));
end


%% ==================== 局部函数 ====================

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