%% ========== DWA + 复杂形状障碍物（正弦运动，彻底无残影） ==========
clear; clc; close all;

%% 1. 机器人参数
robot.v_max = 0.3;
robot.v_min = 0.0;
robot.w_max = 20*pi/180;
robot.w_min = -20*pi/180;
robot.a_max = 0.3;
robot.alpha_max = 40*pi/180;
robot.dv = 0.05;
robot.dw = 1.0*pi/180;
robot.dt = 0.1;
robot.predict_time = 3.0;
robot.radius = 0.3;

weights = [0.10, 0.25, 0.45];

pose = [0, 0, pi/2];
vel  = [0, 0];
goal = [10, 10];

%% 2. 障碍物定义（正弦运动）
obs(1).type = 'circle';
obs(1).shape_params = 0.8;
obs(1).motion.center_x = 2.0;  obs(1).motion.center_y = 2.0;
obs(1).motion.amp_x = 1;     obs(1).motion.amp_y = 0.0;
obs(1).motion.freq_x = 0.2;    obs(1).motion.freq_y = 0.2;
obs(1).motion.phase_x = 0;     obs(1).motion.phase_y = 0;

obs(2).type = 'rectangle';
obs(2).shape_params = [2.0, 1.2];
obs(2).motion.center_x = 5.0;  obs(2).motion.center_y = 3.0;
obs(2).motion.amp_x = 0.0;     obs(2).motion.amp_y = 0.2;
obs(2).motion.freq_x = 0.25;   obs(2).motion.freq_y = 0.25;
obs(2).motion.phase_x = 0;     obs(2).motion.phase_y = pi/2;

obs(3).type = 'polygon';
n = 6; ang = linspace(0,2*pi,n+1)'; ang(end)=[];
r = 0.8;
obs(3).shape_params = [r*cos(ang), r*sin(ang)];
obs(3).motion.center_x = 8.0;  obs(3).motion.center_y = 2.0;
obs(3).motion.amp_x = 0.8;     obs(3).motion.amp_y = 0.8;
obs(3).motion.freq_x = 0.15;   obs(3).motion.freq_y = 0.15;
obs(3).motion.phase_x = 0;     obs(3).motion.phase_y = pi/2;

obs(4).type = 'circle';
obs(4).shape_params = 0.3;
obs(4).motion.center_x = 4.0;  obs(4).motion.center_y = 7.0;
obs(4).motion.amp_x = 1.0;     obs(4).motion.amp_y = 0.5;
obs(4).motion.freq_x = 0.3;    obs(4).motion.freq_y = 0.2;
obs(4).motion.phase_x = 0;     obs(4).motion.phase_y = pi/3;

obs(5).type = 'rectangle';
obs(5).shape_params = [0.8, 1.8];
obs(5).motion.center_x = 7.0;  obs(5).motion.center_y = 8.0;
obs(5).motion.amp_x = 1.2;     obs(5).motion.amp_y = 0.0;
obs(5).motion.freq_x = 0.18;   obs(5).motion.freq_y = 0.18;
obs(5).motion.phase_x = pi/4;  obs(5).motion.phase_y = 0;

% 预计算子圆集合
max_circles = 20;
obs_circles = cell(1, length(obs));
for k = 1:length(obs)
    obs_circles{k} = shape_to_circles(obs(k), max_circles);
end

% 历史轨迹（环形缓冲，最多保存200个点）
MAX_TRAJ = 500;
traj_buffer = zeros(MAX_TRAJ, 2);
traj_idx = 1;
traj_count = 0;

%% 3. 仿真主循环
max_iter = 1000;
goal_tolerance = 0.4;
t = 0;

% 创建图形窗口并设置渲染
fig = figure('Renderer', 'opengl', 'DoubleBuffer', 'on');
axis equal; grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)');
title('DWA 正弦障碍物 ');

for i = 1:max_iter
    dist_goal = norm(pose(1:2) - goal);
    if dist_goal < goal_tolerance
        fprintf('✅ 到达目标！步数 %d\n', i);
        break;
    end
    
    % --- 1) 障碍物正弦运动 ---
    t = t + robot.dt;
    obs_pos = zeros(length(obs), 2);
    for k = 1:length(obs)
        m = obs(k).motion;
        x = m.center_x + m.amp_x * sin(0.2*pi*m.freq_x * t + m.phase_x);
        y = m.center_y + m.amp_y * sin(0.2*pi*m.freq_y * t + m.phase_y);
        obs_pos(k,:) = [x, y];
        obs(k).x = x;
        obs(k).y = y;
    end
    
    % 更新子圆集合
    cur_circles = cell(1, length(obs));
    for k = 1:length(obs)
        base = obs_circles{k}(:,1:2);
        radii = obs_circles{k}(:,3);
        cur_circles{k} = [base + obs_pos(k,:), radii];
    end
    
    % --- 2) DWA 控制 ---
    dw = calc_dynamic_window(vel, robot);
    [best_v, best_w, best_traj, best_score] = ...
        dwa_control(pose, vel, dw, goal, cur_circles, robot, weights);
    
    vel = [best_v, best_w];
    pose = update_pose(pose, vel, robot.dt);
    
    % 更新轨迹环形缓冲
    traj_buffer(traj_idx,:) = pose(1:2);
    traj_idx = traj_idx + 1;
    if traj_idx > MAX_TRAJ
        traj_idx = 1;
    end
    traj_count = min(traj_count + 1, MAX_TRAJ);
    
    % --- 3) 可视化（clf 彻底重建） ---
    if mod(i, 5) == 0 || i == 1
        clf;   % 清除整个图形窗口
        hold on; axis equal; grid on;
        xlabel('X (m)'); ylabel('Y (m)');
        title(sprintf('DWA 正弦障碍物 - 步数 %d', i));
        
        % 目标点
        plot(goal(1), goal(2), 'g*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', '目标点');
        
        % 障碍物（第一个显示图例）
        for k = 1:length(obs)
            if k == 1
                draw_obstacle_shape(obs(k), true);
            else
                draw_obstacle_shape(obs(k), false);
            end
        end
        
        % 绘制历史路径（从环形缓冲中提取有效轨迹）
        if traj_count > 1
            % 提取有效点（按顺序）
            if traj_count == MAX_TRAJ
                % 满环，从 traj_idx 到末尾，再到开头
                idx = [traj_idx:MAX_TRAJ, 1:traj_idx-1];
            else
                % 未满，从头开始
                idx = 1:traj_count;
            end
            valid_traj = traj_buffer(idx, :);
            plot(valid_traj(:,1), valid_traj(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', '路径');
        end
        
        % 机器人
        draw_robot(pose, robot.radius);
        
        % 最优预测轨迹
        if ~isempty(best_traj)
            plot(best_traj(:,1), best_traj(:,2), 'm--', 'LineWidth', 1.5, 'DisplayName', '最优轨迹');
        end
        
        legend('show');
        drawnow expose;   % 强制立即刷新
        pause(0.001);
    end
end

if dist_goal >= goal_tolerance
    fprintf('❌ 未达目标，最终距离 %.3f\n', dist_goal);
end


%% ====================== 辅助函数 ======================

function circles = shape_to_circles(obs, max_circles)
    switch obs.type
        case 'circle'
            r = obs.shape_params;
            circles = [0, 0, r];
        case 'rectangle'
            w = obs.shape_params(1); h = obs.shape_params(2);
            nx = max(2, ceil(sqrt(max_circles * w / h)));
            ny = max(2, ceil(sqrt(max_circles * h / w)));
            if nx*ny > max_circles
                ratio = sqrt(max_circles/(nx*ny));
                nx = max(2, round(nx*ratio));
                ny = max(2, round(ny*ratio));
            end
            x_pts = linspace(-w/2, w/2, nx);
            y_pts = linspace(-h/2, h/2, ny);
            [X, Y] = meshgrid(x_pts, y_pts);
            centers = [X(:), Y(:)];
            if length(x_pts) > 1, dx = x_pts(2)-x_pts(1); else dx = w/2; end
            if length(y_pts) > 1, dy = y_pts(2)-y_pts(1); else dy = h/2; end
            r_sub = min(dx, dy)/2*0.9;
            circles = [centers, r_sub*ones(size(centers,1),1)];
        case 'polygon'
            verts = obs.shape_params;
            center = mean(verts,1);
            pts = [center; verts];
            dists = sqrt(sum(verts.^2,2));
            r_sub = min(dists)*0.4;
            circles = [pts, r_sub*ones(size(pts,1),1)];
            if size(circles,1) > max_circles
                idx = randperm(size(circles,1), max_circles);
                circles = circles(idx,:);
            end
    end
end

function h = draw_obstacle_shape(obs, show_legend)
    if nargin < 2, show_legend = false; end
    x = obs.x; y = obs.y;
    switch obs.type
        case 'circle'
            r = obs.shape_params;
            th = linspace(0,2*pi,40);
            cx = x + r*cos(th); cy = y + r*sin(th);
            if show_legend
                h = fill(cx, cy, 'r', 'EdgeColor','k','LineWidth',1,'DisplayName','障碍物');
            else
                h = fill(cx, cy, 'r', 'EdgeColor','k','LineWidth',1,'HandleVisibility','off');
            end
        case 'rectangle'
            w = obs.shape_params(1); h_rect = obs.shape_params(2);
            rx = [x-w/2, x+w/2, x+w/2, x-w/2];
            ry = [y-h_rect/2, y-h_rect/2, y+h_rect/2, y+h_rect/2];
            if show_legend
                h = fill(rx, ry, 'r', 'EdgeColor','k','LineWidth',1,'DisplayName','障碍物');
            else
                h = fill(rx, ry, 'r', 'EdgeColor','k','LineWidth',1,'HandleVisibility','off');
            end
        case 'polygon'
            verts = obs.shape_params;
            gv = verts + [x, y];
            if show_legend
                h = fill(gv(:,1), gv(:,2), 'r', 'EdgeColor','k','LineWidth',1,'DisplayName','障碍物');
            else
                h = fill(gv(:,1), gv(:,2), 'r', 'EdgeColor','k','LineWidth',1,'HandleVisibility','off');
            end
    end
end

function dw = calc_dynamic_window(vel, robot)
    v = vel(1); w = vel(2);
    dt = robot.dt;
    v_min = max(v - robot.a_max*dt, robot.v_min);
    v_max = min(v + robot.a_max*dt, robot.v_max);
    w_min = max(w - robot.alpha_max*dt, robot.w_min);
    w_max = min(w + robot.alpha_max*dt, robot.w_max);
    v_min = max(v_min, 0.1);
    dw = [v_min, v_max, w_min, w_max];
end

function [best_v, best_w, best_traj, best_score] = ...
    dwa_control(pose, vel, dw, goal, circles_set, robot, weights)
    v_min = dw(1); v_max = dw(2);
    w_min = dw(3); w_max = dw(4);
    v_samples = v_min : robot.dv : v_max;
    w_samples = w_min : robot.dw : w_max;
    max_samples = 500;
    if length(v_samples)*length(w_samples) > max_samples
        nv = ceil(sqrt(max_samples));
        nw = ceil(sqrt(max_samples));
        v_idx = round(linspace(1, length(v_samples), nv));
        w_idx = round(linspace(1, length(w_samples), nw));
        v_samples = v_samples(v_idx);
        w_samples = w_samples(w_idx);
    end
    best_score = -inf;
    best_v = vel(1); best_w = vel(2); best_traj = [];
    for v = v_samples
        for w = w_samples
            traj = generate_trajectory(pose, [v,w], robot.dt, robot.predict_time);
            score = evaluate_trajectory(traj, goal, circles_set, v, robot, weights);
            if score > best_score
                best_score = score;
                best_v = v; best_w = w; best_traj = traj;
            end
        end
    end
    if best_score < 0.05
        dx = goal(1)-pose(1); dy = goal(2)-pose(2);
        target_angle = atan2(dy, dx);
        angle_err = normalize_angle(target_angle - pose(3));
        best_v = min(0.5, robot.v_max);
        best_w = max(min(angle_err*2.0, robot.w_max), robot.w_min);
        best_traj = generate_trajectory(pose, [best_v,best_w], robot.dt, robot.predict_time);
    end
end

function traj = generate_trajectory(pose, vel, dt, predict_time)
    x=pose(1); y=pose(2); theta=pose(3);
    v=vel(1); w=vel(2);
    n = max(1, ceil(predict_time/dt));
    traj = zeros(n,3);
    for i=1:n
        x = x + v*dt*cos(theta);
        y = y + v*dt*sin(theta);
        theta = theta + w*dt;
        traj(i,:) = [x,y,theta];
    end
end

function score = evaluate_trajectory(traj, goal, circles_set, v, robot, weights)
    last = traj(end,:);
    dx = goal(1)-last(1); dy = goal(2)-last(2);
    goal_angle = atan2(dy, dx);
    heading_err = abs(normalize_angle(goal_angle - last(3)));
    heading_score = max(0, 1 - heading_err/pi);
    
    min_effective = inf;
    for k = 1:length(circles_set)
        circles = circles_set{k};
        if isempty(circles), continue; end
        traj_xy = traj(:,1:2);
        for i = 1:size(traj_xy,1)
            pt = traj_xy(i,:);
            d = sqrt((pt(1)-circles(:,1)).^2 + (pt(2)-circles(:,2)).^2);
            effective = d - circles(:,3) - robot.radius;
            min_effective = min(min_effective, min(effective));
        end
    end
    if isinf(min_effective)
        dist_score = 1.0;
    else
        if min_effective < 0, dist_score = 0;
        elseif min_effective > 1.0, dist_score = 1.0;
        else dist_score = min_effective / 1.0;
        end
    end
    if v > 0.05, vel_score = v / robot.v_max;
    else vel_score = -0.2;
    end
    score = weights(1)*heading_score + weights(2)*dist_score + weights(3)*vel_score + 0.01;
end

function new_pose = update_pose(pose, vel, dt)
    x = pose(1) + vel(1)*dt*cos(pose(3));
    y = pose(2) + vel(1)*dt*sin(pose(3));
    theta = pose(3) + vel(2)*dt;
    new_pose = [x,y,theta];
end

function a = normalize_angle(a)
    a = mod(a+pi, 2*pi) - pi;
end

function h = draw_robot(pose, radius)
    x=pose(1); y=pose(2); theta=pose(3);
    th = 0:0.05:2*pi;
    cx = x + radius*cos(th);
    cy = y + radius*sin(th);
    h = fill(cx, cy, 'b', 'EdgeColor','k', 'DisplayName','机器人');
    arrow_len = radius*1.5;
    plot([x, x+arrow_len*cos(theta)], [y, y+arrow_len*sin(theta)], 'r-','LineWidth',2,'HandleVisibility','off');
end