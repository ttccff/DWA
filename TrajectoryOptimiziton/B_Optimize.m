clear; clc; close all;

%% 给定点
pts = [
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

p = 3;                     % B 样条次数，3 为三次 B 样条

% mode = 1: 直接把给定点当作B样条控制点
% mode = 2: 插值，反求控制点，使曲线严格经过给定点
mode = 1;

if mode == 1
    %% 方法1：给定点作为 B 样条控制点
    P = pts;
    n = size(P, 1) - 1;

    % 构造 clamped 节点向量
    U = [zeros(1, p+1), (1:(n-p))/(n-p+1), ones(1, p+1)];

    label = '三次B样条曲线（给定点作为控制点）';

elseif mode == 2
    %% 方法2：B 样条插值，曲线严格通过所有给定点
    Q = pts;
    m = size(Q, 1) - 1;

    % 弦长参数化
    t = zeros(m+1, 1);
    for i = 2:m+1
        t(i) = t(i-1) + norm(Q(i, :) - Q(i-1, :));
    end
    t = t / t(end);

    % 构造插值用节点向量，使用均值法
    U = zeros(1, m+p+2);
    U(1:p+1) = 0;
    for j = 1:m-p
        U(j+p+1) = mean(t(j+1:j+p));
    end
    U(end-p:end) = 1;

    % 构造配置矩阵
    A = zeros(m+1, m+1);
    for i = 1:m+1
        A(i, :) = bspline_basis_all(t(i), U, p, m);
    end

    % 反求 B 样条控制点
    Px = A \ Q(:, 1);
    Py = A \ Q(:, 2);
    P = [Px, Py];

    n = m;
    label = '三次B样条插值曲线（严格通过所有给定点）';

else
    error('mode 只能为 1 或 2');
end

%% 计算并绘制 B 样条曲线
u = linspace(0, 1, 500);
C = zeros(length(u), 2);

for i = 1:length(u)
    N = bspline_basis_all(u(i), U, p, n);
    C(i, :) = N * P;
end

figure('Name', 'B样条拟合结果');
hold on; grid on; axis equal;
dim = size(pts, 2);
if dim >= 3
    view(3);
    plot3(pts(:,1), pts(:,2), pts(:,3), 'k--', 'LineWidth', 1.0, 'DisplayName', '原始折线');
    plot3(pts(:,1), pts(:,2), pts(:,3), 'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r', 'DisplayName', '插值参考点');
    plot3(C(:,1), C(:,2), C(:,3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'B样条曲线');
    xlabel('X'); ylabel('Y'); zlabel('Z');
else
    plot(pts(:,1), pts(:,2), 'k--', 'LineWidth', 1.0, 'DisplayName', '原始折线');
    plot(pts(:,1), pts(:,2), 'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r', 'DisplayName', '插值参考点');
    plot(C(:,1), C(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'B样条曲线');
    xlabel('X'); ylabel('Y');
end
legend('show');

%% 局部函数：计算所有 B 样条基函数
function N = bspline_basis_all(u, U, p, n)
% 返回 N(1:n+1)，对应 N_{0,p}(u), ..., N_{n,p}(u)

N = zeros(1, n+1);

% 寻找 u 所在节点区间 k，0-based 索引
if u >= U(n+2)          % U_{n+1}，到达右端点
    k = n;
else
    k = p;
    for idx = p:n
        if u >= U(idx+1) && u < U(idx+2)
            k = idx;
            break;
        end
    end
end

% de Boor-Cox 递推计算非零基函数
left = zeros(1, p);
right = zeros(1, p);
Nloc = zeros(1, p+1);
Nloc(1) = 1;

for j = 1:p
    left(j) = u - U(k - j + 2);     % U_{k+1-j}
    right(j) = U(k + j + 1) - u;    % U_{k+j}

    saved = 0;
    for r = 0:j-1
        temp = Nloc(r+1) / (right(r+1) + left(j-r));
        Nloc(r+1) = saved + right(r+1) * temp;
        saved = left(j-r) * temp;
    end
    Nloc(j+1) = saved;
end

% 映射回全局基函数索引
for j = 1:p+1
    global_idx = k - p + (j - 1);
    if global_idx >= 0 && global_idx <= n
        N(global_idx + 1) = Nloc(j);
    end
end

end