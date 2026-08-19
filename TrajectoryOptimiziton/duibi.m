clc; clear; close all;

%% ==================== 参数设置 ====================
waypoints_original = [
     0,  0;      % 起点
     5,  0;      % 长直线段
     6,  0.8;    % 小角度转弯（约 14°）
     5,  2.5;    % 大角度转弯（约 120°）
     2,  2.5;    % 90° 直角转弯
     0,  4;      % 约 135° 钝角转弯
    -2,  4;      % 90° 直角转弯
    -3,  3;      % 接近 180° 掉头（约 153°）
    -3,  0;      % 90° 直角转弯
];

% 每个原始段之间插入的内部点数
n_interp = 0;

% 轨迹多项式阶数
N = 7;

% 中间点连续导数最高阶：位置、速度、加速度、jerk、snap 连续
cont_order = 4;

% 混合权重：最小 SNAP 与最小 JERK 各 0.5
w_snap = 0.1;
w_jerk = 0.9;

% 梯形时间分配参数
max_vel_trap = 5;
max_accel_trap = 1;

% 转向角时间分配参数
% alpha_turn：缩短系数（0~1），拐角小于50°时缩短时间
% beta_angle：增加系数（>=0），拐角大于50°时增加时间
alpha_turn = 0.2;   % 缩短最多30%
beta_angle = 0.6;   % 增加最多60%

%% ==================== 参考点插值 ====================
m_orig = size(waypoints_original, 1);
waypoints = waypoints_original(1, :);   % 先添加起点

for i = 1:m_orig-1
    p_start = waypoints_original(i, :);
    p_end   = waypoints_original(i+1, :);
    for j = 1:n_interp
        alpha = j / (n_interp + 1);
        waypoints = [waypoints; (1-alpha)*p_start + alpha*p_end]; %#ok<AGROW>
    end
    waypoints = [waypoints; p_end]; %#ok<AGROW>
end

%% ==================== 时间分配 ====================
% 梯形时间分配
T_trap = allocate_time_by_max_vel_accel(waypoints, max_vel_trap, max_accel_trap);

% 梯形 + 转向角时间分配（在梯形基础上修正，总时间可变）
T_turn = allocate_time_by_turning_angle(waypoints, T_trap, alpha_turn, beta_angle);

% 平均时间分配：总时间与梯形分配相同
K = size(waypoints, 1) - 1;
T_fixed = ones(1, K) * (sum(T_trap) / K);

% 时间分配柱状图
figure('Name', '时间分配对比');
bar_data = [T_fixed(:)'; T_trap(:)'; T_turn(:)']';
bar(bar_data, 0.8);
xlabel('段索引');
ylabel('时间 (s)');
title('每段轨迹时间分配对比');
legend('固定均分', '梯形分配', '梯形+转向角', 'Location', 'best');
grid on;

%% ==================== 构造二次型矩阵 ====================
% 转向角时间分配下
Q_snap_turn  = build_Q_for_derivative(N, T_turn, 4);
Q_jerk_turn  = build_Q_for_derivative(N, T_turn, 3);
Q_mixed_turn = w_snap * Q_snap_turn + w_jerk * Q_jerk_turn;

% 平均时间分配下
Q_snap_fixed  = build_Q_for_derivative(N, T_fixed, 4);
Q_jerk_fixed  = build_Q_for_derivative(N, T_fixed, 3);
Q_mixed_fixed = w_snap * Q_snap_fixed + w_jerk * Q_jerk_fixed;

%% ==================== 求解轨迹系数 ====================
% 转向角时间分配
coeffs_snap_turn  = compute_coeffs(waypoints, N, T_turn, cont_order, Q_snap_turn);
coeffs_mixed_turn = compute_coeffs(waypoints, N, T_turn, cont_order, Q_mixed_turn);

% 平均时间分配
coeffs_snap_fixed  = compute_coeffs(waypoints, N, T_fixed, cont_order, Q_snap_fixed);
coeffs_mixed_fixed = compute_coeffs(waypoints, N, T_fixed, cont_order, Q_mixed_fixed);

%% ==================== 图1：最小SNAP 与 最小SNAP+最小JERK结合对比 ====================
total_time_turn = sum(T_turn);
ts_turn = linspace(0, total_time_turn, 500);

traj_snap_turn  = evaluate_trajectory_derivative(coeffs_snap_turn,  N, T_turn, ts_turn, 0);
traj_mixed_turn = evaluate_trajectory_derivative(coeffs_mixed_turn, N, T_turn, ts_turn, 0);

dim = size(waypoints, 2);

figure('Name', '图1：最小SNAP 与 最小SNAP+最小JERK结合对比');
hold on; grid on; axis equal;
if dim >= 3
    view(3);
    plot3(waypoints_original(:,1), waypoints_original(:,2), waypoints_original(:,3), ...
        'k--', 'LineWidth', 1.0, 'DisplayName', '原始折线');
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), ...
        'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r', 'DisplayName', '插值参考点');
    plot3(traj_snap_turn(:,1), traj_snap_turn(:,2), traj_snap_turn(:,3), ...
        'b-', 'LineWidth', 1.5, 'DisplayName', '最小SNAP');
    plot3(traj_mixed_turn(:,1), traj_mixed_turn(:,2), traj_mixed_turn(:,3), ...
        'r--', 'LineWidth', 1.5, 'DisplayName', '最小SNAP+最小JERK (0.5/0.5)');
    xlabel('X'); ylabel('Y'); zlabel('Z');
else
    plot(waypoints_original(:,1), waypoints_original(:,2), ...
        'k--', 'LineWidth', 1.0, 'DisplayName', '原始折线');
    plot(waypoints(:,1), waypoints(:,2), ...
        'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r', 'DisplayName', '插值参考点');
    plot(traj_snap_turn(:,1), traj_snap_turn(:,2), ...
        'b-', 'LineWidth', 1.5, 'DisplayName', '最小SNAP');
    plot(traj_mixed_turn(:,1), traj_mixed_turn(:,2), ...
        'r--', 'LineWidth', 1.5, 'DisplayName', '最小SNAP+最小JERK (0.5/0.5)');
    xlabel('X'); ylabel('Y');
end
legend('Location', 'best');
title('图1：相同参考点下最小SNAP与最小SNAP+最小JERK结合轨迹对比');
hold off;

%% ==================== 图2：平均时间分配 与 梯形+拐角时间分配对比 ====================
ts_fixed = linspace(0, sum(T_fixed), 500);

traj_mixed_fixed = evaluate_trajectory_derivative(coeffs_mixed_fixed, N, T_fixed, ts_fixed, 0);
traj_mixed_turn  = evaluate_trajectory_derivative(coeffs_mixed_turn,  N, T_turn,  ts_turn,  0);

figure('Name', '图2：平均时间分配 与 梯形+拐角时间分配对比');
hold on; grid on; axis equal;
if dim >= 3
    view(3);
    plot3(waypoints_original(:,1), waypoints_original(:,2), waypoints_original(:,3), ...
        'k--', 'LineWidth', 1.0, 'DisplayName', '原始折线');
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), ...
        'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r', 'DisplayName', '插值参考点');
    plot3(traj_mixed_fixed(:,1), traj_mixed_fixed(:,2), traj_mixed_fixed(:,3), ...
        'b-', 'LineWidth', 1.5, 'DisplayName', '平均时间分配-混合优化');
    plot3(traj_mixed_turn(:,1), traj_mixed_turn(:,2), traj_mixed_turn(:,3), ...
        'r--', 'LineWidth', 1.5, 'DisplayName', '梯形+拐角时间分配-混合优化');
    xlabel('X'); ylabel('Y'); zlabel('Z');
else
    plot(waypoints_original(:,1), waypoints_original(:,2), ...
        'k--', 'LineWidth', 1.0, 'DisplayName', '原始折线');
    plot(waypoints(:,1), waypoints(:,2), ...
        'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r', 'DisplayName', '插值参考点');
    plot(traj_mixed_fixed(:,1), traj_mixed_fixed(:,2), ...
        'b-', 'LineWidth', 1.5, 'DisplayName', '平均时间分配-混合优化');
    plot(traj_mixed_turn(:,1), traj_mixed_turn(:,2), ...
        'r--', 'LineWidth', 1.5, 'DisplayName', '梯形+拐角时间分配-混合优化');
    xlabel('X'); ylabel('Y');
end
legend('Location', 'best');
title('图2：平均时间分配与梯形+拐角时间分配轨迹对比（混合优化）');
hold off;

%% ==================== 新增：不同策略的动态曲线对比 ====================
% 计算各阶导数数据
% 优化目标对比（转向角时间分配）
deriv_orders = 0:4;  % 位置、速度、加速度、jerk、snap
traj_snap_turn_all  = cell(1,5);
traj_mixed_turn_all = cell(1,5);
for idx = 1:5
    traj_snap_turn_all{idx}  = evaluate_trajectory_derivative(coeffs_snap_turn,  N, T_turn, ts_turn, deriv_orders(idx));
    traj_mixed_turn_all{idx} = evaluate_trajectory_derivative(coeffs_mixed_turn, N, T_turn, ts_turn, deriv_orders(idx));
end

% 时间分配对比（混合优化）
traj_mixed_fixed_all = cell(1,5);
traj_mixed_turn_all2 = cell(1,5);
for idx = 1:5
    traj_mixed_fixed_all{idx} = evaluate_trajectory_derivative(coeffs_mixed_fixed, N, T_fixed, ts_fixed, deriv_orders(idx));
    traj_mixed_turn_all2{idx} = evaluate_trajectory_derivative(coeffs_mixed_turn, N, T_turn, ts_turn, deriv_orders(idx));
end

% 方向名称
direction_names = {'X', 'Y', 'Z'};

% 导数名称
deriv_names = {'位置', '速度', '加速度', 'Jerk', 'Snap'};

%% 图3：优化目标对比（最小SNAP vs 混合）各阶导数曲线
figure('Name', '图3：优化目标对比（最小SNAP vs 混合）');
for d = 1:min(dim,2)  % 只处理X和Y方向（2D路径）
    dir_name = direction_names{d};
    for r = 1:5
        subplot(5,2,(r-1)*2 + d);
        plot(ts_turn, traj_snap_turn_all{r}(:,d), 'b-', 'LineWidth', 1.2); hold on;
        plot(ts_turn, traj_mixed_turn_all{r}(:,d), 'r--', 'LineWidth', 1.2);
        xlabel('时间 (s)');
        ylabel(deriv_names{r});
        title(sprintf('%s 方向 %s', dir_name, deriv_names{r}));
        grid on;
        if r == 1
            legend('最小SNAP', '混合优化', 'Location', 'best');
        end
    end
end
sgtitle('图3：优化目标对比（转向角时间分配下）');

%% 图4：时间分配对比（平均 vs 梯形+拐角）各阶导数曲线
figure('Name', '图4：时间分配对比（平均 vs 梯形+拐角）');
for d = 1:min(dim,2)
    dir_name = direction_names{d};
    for r = 1:5
        subplot(5,2,(r-1)*2 + d);
        plot(ts_fixed, traj_mixed_fixed_all{r}(:,d), 'b-', 'LineWidth', 1.2); hold on;
        plot(ts_turn, traj_mixed_turn_all2{r}(:,d), 'r--', 'LineWidth', 1.2);
        xlabel('时间 (s)');
        ylabel(deriv_names{r});
        title(sprintf('%s 方向 %s', dir_name, deriv_names{r}));
        grid on;
        if r == 1
            legend('平均时间分配', '梯形+拐角', 'Location', 'best');
        end
    end
end
sgtitle('图4：时间分配对比（混合优化下）');

%% ==================== 局部函数 ====================

% 梯形时间分配：基于最大速度和最大加速度
function T = allocate_time_by_max_vel_accel(waypoints, max_vel, max_accel)
    diffs = diff(waypoints, 1, 1);
    dists = sqrt(sum(diffs.^2, 2))';   % 行向量

    t = max_vel / max_accel;                        % 加速到最大速度所需时间
    dist_threshold = max_accel * t^2;               % 加速+减速所需总距离

    K = size(waypoints, 1) - 1;
    T = zeros(1, K);
    for i = 1:K
        delta_dist = dists(i);
        if delta_dist > dist_threshold
            T(i) = 2 * t + (delta_dist - dist_threshold) / max_vel;
        else
            T(i) = 2 * sqrt(delta_dist / max_accel);
        end
    end
end

% 基于转向角的时间分配函数（在梯形时间基础上修正，总时间可变）
% 修改说明：拐角小于50°缩短时间，大于50°增加时间
function T = allocate_time_by_turning_angle(waypoints, T_base, alpha_shorten, beta_lengthen)
    % waypoints: 航点矩阵，每行一个点
    % T_base:    梯形时间分配得到的各段时间，行向量
    % alpha_shorten: 缩短系数（0~1），拐角小于阈值时缩短时间
    % beta_lengthen: 增加系数（>=0），拐角大于阈值时增加时间
    % 返回: 在梯形时间基础上根据转弯角度修正后的各段时间（总时间不约束）

    N = size(waypoints, 1);
    K = N - 1;

    % 检查 T_base 长度是否匹配
    if length(T_base) ~= K
        error('T_base 长度必须等于航点段数 K');
    end

    % 设置角度阈值：50度
    theta_threshold = deg2rad(50);

    % 计算每个中间点的转向角（弧度）
    angles = zeros(1, N);
    for i = 2:N-1
        v1 = waypoints(i, :) - waypoints(i-1, :);
        v2 = waypoints(i+1, :) - waypoints(i, :);
        v1_norm = norm(v1);
        v2_norm = norm(v2);
        if v1_norm > 1e-9 && v2_norm > 1e-9
            cos_theta = dot(v1, v2) / (v1_norm * v2_norm);
            cos_theta = max(-1, min(1, cos_theta));
            angles(i) = acos(cos_theta);
        else
            angles(i) = 0;
        end
    end

    % 对每一段，取两端点平均角度，计算修正系数
    factors = ones(1, K);
    for i = 1:K
        theta_avg = (angles(i) + angles(i+1)) / 2;  % 弧度

        if theta_avg < theta_threshold
            % 小于阈值：缩短时间
            ratio = theta_avg / theta_threshold;   % 0~1
            factors(i) = 1 - alpha_shorten * (1 - ratio)*0.5;
        else
            % 大于等于阈值：延长时间
            ratio = (theta_avg - theta_threshold) / (pi - theta_threshold); % 0~1
            factors(i) = 1 + beta_lengthen * ratio;
        end

        % 限制因子范围，避免时间过短或过长
        factors(i) = max(0.3, min(2.0, factors(i)));
    end

    % 在梯形时间基础上乘以修正系数（不再缩放总时间）
    T = T_base .* factors;
end

% 构造某个导数阶数对应的二次型矩阵
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
function [Aeq, beq] = build_constraints(wp, N, T_segments, cont_order)
    m = length(wp);
    K = m - 1;
    n = K * (N + 1);

    pos_rows   = 2 * m - 2;
    cont_rows  = cont_order * (m - 2);
    start_rows = cont_order;
    end_rows   = cont_order;
    total_rows = pos_rows + cont_rows + start_rows + end_rows;

    Aeq = zeros(total_rows, n);
    beq = zeros(total_rows, 1);
    row_idx = 0;

    % 起点位置
    row_idx = row_idx + 1;
    Aeq(row_idx, 1:(N + 1)) = derivative_row(N, 0, 0);
    beq(row_idx) = wp(1);

    % 中间点位置：上一段末端和下一段起始
    for j = 2:m - 1
        prev_seg = j - 1;
        next_seg = j;
        T_prev = T_segments(prev_seg);
        idx_prev = (prev_seg - 1) * (N + 1) + 1 : prev_seg * (N + 1);
        idx_next = (next_seg - 1) * (N + 1) + 1 : next_seg * (N + 1);

        row_idx = row_idx + 1;
        Aeq(row_idx, idx_prev) = derivative_row(N, 0, T_prev);
        beq(row_idx) = wp(j);

        row_idx = row_idx + 1;
        Aeq(row_idx, idx_next) = derivative_row(N, 0, 0);
        beq(row_idx) = wp(j);
    end

    % 终点位置
    row_idx = row_idx + 1;
    idx_last = (K - 1) * (N + 1) + 1 : K * (N + 1);
    Aeq(row_idx, idx_last) = derivative_row(N, 0, T_segments(K));
    beq(row_idx) = wp(m);

    % 中间点连续导数约束
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

    % 起点高阶导数置零
    for r = 1:cont_order
        row_idx = row_idx + 1;
        Aeq(row_idx, 1:(N + 1)) = derivative_row(N, r, 0);
        beq(row_idx) = 0;
    end

    % 终点高阶导数置零
    for r = 1:cont_order
        row_idx = row_idx + 1;
        Aeq(row_idx, idx_last) = derivative_row(N, r, T_segments(K));
        beq(row_idx) = 0;
    end
end

% 求多项式第 r 阶导数在时间 t 处的系数行向量
function row = derivative_row(N, r, t)
    row = zeros(1, N + 1);
    if r > N
        return;
    end
    for k = r:N
        row(k + 1) = factorial(k) / factorial(k - r) * t^(k - r);
    end
end

% 求解等式约束二次规划（KKT 方法）
function coeffs = solve_eq_qp(Q, Aeq, beq)
    n = size(Q, 1);
    m = size(Aeq, 1);
    KKT = [Q, Aeq'; Aeq, zeros(m, m)];
    rhs = [zeros(n, 1); beq];
    if rcond(KKT) < 1e-12
        warning('KKT 矩阵接近奇异，加入小正则化');
        KKT = KKT + 1e-8 * eye(size(KKT));
    end
    sol = KKT \ rhs;
    coeffs = sol(1:n);
end

% 对每个维度分别求解轨迹系数（无不等式约束）
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