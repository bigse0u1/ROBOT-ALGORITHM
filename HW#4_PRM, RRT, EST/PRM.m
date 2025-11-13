clear; close all; clc;
rng(0,'twister');

%환경 / 장애물 / 로봇 정의
env.x_min = 0;  env.x_max = 20;
env.y_min = 0;  env.y_max = 20;

obs{1} = polyshape([3 3 8 8],     [3 8 8 3]);
obs{2} = polyshape([5 5 6 6],     [12 18 18 12]);
obs{3} = polyshape([12 12 18 18], [10 16 16 10]);
obs{4} = polyshape([13 13 15 15], [2 8 8 2]);

% L-자 로봇 모양
L_w = 1.5; L_h = 1.5; L_t = 0.5;

rect1 = polyshape( ...
    [-L_w/2, -L_w/2+L_t, -L_w/2+L_t, -L_w/2], ...
    [-L_h/2, -L_h/2,      L_h/2,      L_h/2]);

rect2 = polyshape( ...
    [-L_w/2+L_t,  L_w/2,       L_w/2,      -L_w/2+L_t], ...
    [ L_h/2-L_t,  L_h/2-L_t,   L_h/2,      L_h/2]);

robot.shape = union(rect1, rect2);

% PRM 파라미터
N_nodes      = 300;   % 노드 수
K_per_node   = 10;    % 이웃 개수
interp_steps = 15;    % 걸음거리

% 시작 / 목표
q_init = [1.5, 1.5, 0];
q_goal = [19, 19, pi/2];

% PRM 로드맵 구성
nodes = zeros(N_nodes, 3);
nodes(1,:) = q_init;
nodes(2,:) = q_goal;

adj = Inf(N_nodes);
adj(1:N_nodes+1:end) = 0;

node_cnt = 2;

while node_cnt < N_nodes
    % 랜덤 샘플 (x,y,theta)
    q_rand = [ ...
        rand()*(env.x_max-env.x_min) + env.x_min, ...
        rand()*(env.y_max-env.y_min) + env.y_min, ...
        rand()*2*pi - pi ...
    ];

    % 충돌이면 버림
    if isCollision_L(q_rand, robot, obs, env)
        continue;
    end

    node_cnt = node_cnt + 1;
    nodes(node_cnt,:) = q_rand;

    % 기존 노드들과 거리 계산
    existing  = nodes(1:node_cnt-1,:);
    dists_xy  = vecnorm(existing(:,1:2) - q_rand(1:2), 2, 2);
    [~, idx]  = sort(dists_xy);
    n_connect = min(K_per_node, numel(idx));
    neigh_idx = idx(1:n_connect);

    % k-NN 후보들과 로컬 경로 충돌 체크 후 간선 추가
    for k = 1:n_connect
        j = neigh_idx(k);
        q_nb = existing(j,:);
        if isPathFree_L(q_rand, q_nb, robot, obs, env, interp_steps)
            w = norm(q_rand(1:2) - q_nb(1:2)); 
            adj(node_cnt, j) = w;
            adj(j, node_cnt) = w;
        end
    end
end

% 최단 경로
start_idx = 1;
goal_idx  = 2;

[path_idx, path_cost] = dijkstra_L(adj, start_idx, goal_idx);

if isinf(path_cost)
    warning('경로를 찾지 못했습니다.');
    path_nodes = [];
else
    path_nodes = nodes(path_idx,:);
end

% 결과 시각화 
fig = figure; hold on; axis equal;

% 경계
plot([env.x_min env.x_max env.x_max env.x_min env.x_min], ...
     [env.y_min env.y_min env.y_max env.y_max env.y_min], ...
     'k-','LineWidth',2);

% 장애물
for i = 1:numel(obs)
    plot(obs{i}, 'FaceColor',[0.5 0.5 0.5], 'FaceAlpha',0.7, 'EdgeColor','k');
end

% 로드맵 간선
[r,c] = find(adj > 0 & adj < Inf);
for n = 1:numel(r)
    if r(n) < c(n)
        plot([nodes(r(n),1) nodes(c(n),1)], ...
             [nodes(r(n),2) nodes(c(n),2)], ...
             'Color',[0.8 0.8 1], 'LineWidth',0.5);
    end
end

% 샘플 노드 
if N_nodes > 2
    plot(nodes(3:end,1), nodes(3:end,2), '.', ...
         'Color',[0 0 1], 'MarkerSize',8);
end

% 시작/목표 로봇 모양
rob_s = translate(rotate(robot.shape, rad2deg(q_init(3))), [q_init(1), q_init(2)]);
rob_g = translate(rotate(robot.shape, rad2deg(q_goal(3))), [q_goal(1), q_goal(2)]);
plot(rob_s, 'FaceColor',[0 1 0], 'FaceAlpha',0.7, 'EdgeColor','k');
plot(rob_g, 'FaceColor',[1 0 0], 'FaceAlpha',0.7, 'EdgeColor','k');

% 최단 경로
if ~isempty(path_nodes)
    plot(path_nodes(:,1), path_nodes(:,2), '-o', ...
         'LineWidth',2, 'Color',[1 0 1], ...
         'MarkerFaceColor','m', 'MarkerEdgeColor',[1 0 1]);
end

title(sprintf('PRM'));
xlabel('X'); ylabel('Y');
axis([env.x_min env.x_max env.y_min env.y_max]);
grid on;
hold off;

%% 로봇 이동
if ~isempty(path_nodes)
    animateLRobot(path_nodes, robot, fig);
end

function [path, cost] = dijkstra_L(adj, s, t)
    n    = size(adj,1);
    dist = Inf(n,1);
    prev = zeros(n,1);
    vis  = false(n,1);

    dist(s) = 0;

    for k = 1:n
        u = -1; best = Inf;
        for i = 1:n
            if ~vis(i) && dist(i) < best
                best = dist(i); u = i;
            end
        end
        if u == -1, break; end
        vis(u) = true;
        if u == t, break; end

        for v = 1:n
            if ~vis(v) && ~isinf(adj(u,v))
                alt = dist(u) + adj(u,v);
                if alt < dist(v)
                    dist(v) = alt;
                    prev(v) = u;
                end
            end
        end
    end

    cost = dist(t);
    if isinf(cost)
        path = [];
        return;
    end

    path = [];
    cur = t;
    while cur ~= 0
        path = [cur; path];
        cur = prev(cur);
    end
    if isempty(path) || path(1) ~= s
        path = [];
        cost = Inf;
    end
end

function coll = isCollision_L(q, robot, obs, env)
    % 범위 밖이면 충돌
    if q(1) < env.x_min || q(1) > env.x_max || ...
       q(2) < env.y_min || q(2) > env.y_max
        coll = true; return;
    end

    % 로봇 자세 적용
    rob_q = translate(rotate(robot.shape, rad2deg(q(3))), [q(1), q(2)]);

    % 장애물과 겹치면 충돌
    for i = 1:numel(obs)
        if overlaps(rob_q, obs{i})
            coll = true; return;
        end
    end
    coll = false;
end

function free = isPathFree_L(q1, q2, robot, obs, env, steps)
    free = true;
    th_diff = atan2(sin(q2(3)-q1(3)), cos(q2(3)-q1(3)));

    for s = linspace(0,1,steps)
        q = [(1-s)*q1(1:2) + s*q2(1:2), q1(3) + s*th_diff];
        if isCollision_L(q, robot, obs, env)
            free = false;
            return;
        end
    end
end

function animateLRobot(path, robot, fig)
    figure(fig); hold on;
    h = [];
    nFrames = 15;

    for k = 1:size(path,1)-1
        q1 = path(k,:);
        q2 = path(k+1,:);

        th_diff = atan2(sin(q2(3)-q1(3)), cos(q2(3)-q1(3)));

        for s = linspace(0,1,nFrames)
            q = [(1-s)*q1(1:2) + s*q2(1:2), q1(3) + s*th_diff];

            rob_poly = translate(rotate(robot.shape, rad2deg(q(3))), ...
                                 [q(1), q(2)]);

            if ~isempty(h) && isvalid(h)
                delete(h);
            end
            h = plot(rob_poly, 'FaceColor',[0 0 0.8], ...
                               'FaceAlpha',0.7, ...
                               'EdgeColor','k');

            drawnow;
            pause(0.01);
        end
    end
end
