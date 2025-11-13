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

% RRT 파라미터
max_iter    = 2000;   % 최대 확장 횟수
step_size   = 0.7;    % 걸음 길이
interp_steps = 15;    % 로컬 경로 충돌 체크용 보간
goal_thresh = 1.0;    % goal 근처로 간주하는 거리
goal_bias   = 0.1;    % goal로 직접 샘플링할 확률

% 시작 / 목표
q_init = [1.5, 1.5, 0];
q_goal = [19, 19, pi/2];

%RRT 트리
nodes   = zeros(max_iter+2, 3);  
parent  = zeros(max_iter+2, 1);
nodes(1,:) = q_init;
parent(1)  = 0;
node_cnt   = 1;

goal_idx   = -1;

for it = 1:max_iter
    if rand() < goal_bias
        q_rand = q_goal;
    else
        q_rand = [ ...
            rand()*(env.x_max-env.x_min) + env.x_min, ...
            rand()*(env.y_max-env.y_min) + env.y_min, ...
            rand()*2*pi - pi ...
        ];
        if isCollision_L(q_rand, robot, obs, env)
            continue;
        end
    end

    existing = nodes(1:node_cnt,:);
    d_xy     = vecnorm(existing(:,1:2) - q_rand(1:2), 2, 2);
    [~, idx_min] = min(d_xy);
    q_near   = existing(idx_min,:);

    dir_xy = q_rand(1:2) - q_near(1:2);
    dist_xy = norm(dir_xy);
    if dist_xy < 1e-6
        continue;
    end
    dir_xy = dir_xy / dist_xy;
    step = min(step_size, dist_xy);
    q_new_xy = q_near(1:2) + step * dir_xy;

    dth = atan2(sin(q_rand(3)-q_near(3)), cos(q_rand(3)-q_near(3)));
    alpha = step / max(dist_xy, step_size);  % 0~1 비율
    q_new_th = q_near(3) + alpha * dth;

    q_new = [q_new_xy, q_new_th];

    if ~isPathFree_L(q_near, q_new, robot, obs, env, interp_steps)
        continue;
    end

    node_cnt           = node_cnt + 1;
    nodes(node_cnt,:)  = q_new;
    parent(node_cnt)   = idx_min;

    d_goal_xy = norm(q_new(1:2) - q_goal(1:2));
    if d_goal_xy < goal_thresh
        if isPathFree_L(q_new, q_goal, robot, obs, env, interp_steps)
            node_cnt          = node_cnt + 1;
            nodes(node_cnt,:) = q_goal;
            parent(node_cnt)  = node_cnt-1;
            goal_idx          = node_cnt;
            disp("RRT: goal reached at iteration " + it);
            break;
        end
    end
end

if goal_idx == -1
    warning('RRT: 목표에 도달하지 못했습니다.');
    path_nodes = [];
else
    path_nodes = [];
    cur = goal_idx;
    while cur ~= 0
        path_nodes = [nodes(cur,:); path_nodes];
        cur = parent(cur);
    end
end

%결과 시각화 

fig = figure; hold on; axis equal;

% 경계
plot([env.x_min env.x_max env.x_max env.x_min env.x_min], ...
     [env.y_min env.y_min env.y_max env.y_max env.y_min], ...
     'k-','LineWidth',2);

% 장애물
for i = 1:numel(obs)
    plot(obs{i}, 'FaceColor',[0.5 0.5 0.5], 'FaceAlpha',0.7, 'EdgeColor','k');
end

% RRT 트리
for n = 2:node_cnt
    p = parent(n);
    if p > 0
        plot([nodes(n,1) nodes(p,1)], ...
             [nodes(n,2) nodes(p,2)], ...
             'Color',[0.8 0.8 1], 'LineWidth',0.5);
    end
end

% 시작 / 목표 로봇 모양
rob_s = translate(rotate(robot.shape, rad2deg(q_init(3))), [q_init(1), q_init(2)]);
rob_g = translate(rotate(robot.shape, rad2deg(q_goal(3))), [q_goal(1), q_goal(2)]);
plot(rob_s, 'FaceColor',[0 1 0], 'FaceAlpha',0.7, 'EdgeColor','k');
plot(rob_g, 'FaceColor',[1 0 0], 'FaceAlpha',0.7, 'EdgeColor','k');

% 최종 경로
if ~isempty(path_nodes)
    plot(path_nodes(:,1), path_nodes(:,2), '-o', ...
         'LineWidth',2, 'Color',[1 0 1], ...
         'MarkerFaceColor','m', 'MarkerEdgeColor',[1 0 1]);
end

title('RRT');
xlabel('X'); ylabel('Y');
axis([env.x_min env.x_max env.y_min env.y_max]);
grid on;
hold off;

if ~isempty(path_nodes)
    animateLRobot(path_nodes, robot, fig);
end

function coll = isCollision_L(q, robot, obs, env)
    if q(1) < env.x_min || q(1) > env.x_max || ...
       q(2) < env.y_min || q(2) > env.y_max
        coll = true; return;
    end

    rob_q = translate(rotate(robot.shape, rad2deg(q(3))), [q(1), q(2)]);
    for i = 1:numel(obs)
        if overlaps(rob_q, obs{i})
            coll = true; return;
        end
    end
    coll = false;
end

function free = isPathFree_L(q1, q2, robot, obs, env, steps)
    free = true;
    dth = atan2(sin(q2(3)-q1(3)), cos(q2(3)-q1(3)));
    for s = linspace(0,1,steps)
        q = [(1-s)*q1(1:2) + s*q2(1:2), q1(3) + s*dth];
        if isCollision_L(q, robot, obs, env)
            free = false; return;
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
        dth = atan2(sin(q2(3)-q1(3)), cos(q2(3)-q1(3)));

        for s = linspace(0,1,nFrames)
            q = [(1-s)*q1(1:2) + s*q2(1:2), q1(3) + s*dth];

            rob_poly = translate(rotate(robot.shape, rad2deg(q(3))), [q(1), q(2)]);
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
