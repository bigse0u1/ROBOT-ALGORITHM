clear; close all; clc;
rng(0,'twister');

% 환경 / 장애물 / 로봇 정의

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

% 시작 / 목표
q_init = [1.5, 1.5, 0];
q_goal = [19, 19, pi/2];

%EST 파라미터

max_nodes    = 3000;   % 트리에 최대 노드 수 
max_iter     = 5000;   % 반복
step_size    = 1.5;    % 걸음 길이 
interp_steps = 18;     % 체크 샘플 수
goal_thresh  = 1.5;    % goal 근처 거리 
nbr_radius   = 1.2;    % 밀도 추정용 반경

%% 3. EST 트리 구축

nodes  = zeros(max_nodes, 3);
parent = zeros(max_nodes, 1);

nodes(1,:) = q_init;
parent(1)  = 0;
node_cnt   = 1;

goal_idx = -1;

for it = 1:max_iter

    existing = nodes(1:node_cnt,:);

    nbr_count = zeros(node_cnt,1);
    for i = 1:node_cnt
        d = vecnorm(existing(:,1:2) - existing(i,1:2), 2, 2);
        nbr_count(i) = sum(d < nbr_radius) - 1;  
    end

    w = 1 ./ (1 + nbr_count);
    s = sum(w);
    if s <= 0
        w = ones(node_cnt,1) / node_cnt;
    else
        w = w / s;
    end

    r = rand();
    cum = cumsum(w);
    sel_idx = find(cum >= r, 1, 'first');
    if isempty(sel_idx)
        sel_idx = node_cnt; 
    end
    q_sel = existing(sel_idx,:);

    dir_angle = rand()*2*pi - pi;
    dir_xy    = [cos(dir_angle), sin(dir_angle)];
    q_new_xy  = q_sel(1:2) + step_size * dir_xy;

    dth = (rand()*2*pi - pi) * 0.3;
    q_new_th = q_sel(3) + dth;
    q_new_th = atan2(sin(q_new_th), cos(q_new_th));  

    q_new = [q_new_xy, q_new_th];

    if isCollision_L(q_new, robot, obs, env)
        continue;
    end
    if ~isPathFree_L(q_sel, q_new, robot, obs, env, interp_steps)
        continue;
    end

    if node_cnt >= max_nodes
        warning('EST: max_nodes에 도달했습니다.'); 
        break;
    end
    node_cnt          = node_cnt + 1;
    nodes(node_cnt,:) = q_new;
    parent(node_cnt)  = sel_idx;

    d_goal_xy = norm(q_new(1:2) - q_goal(1:2));
    if d_goal_xy < goal_thresh
        if isPathFree_L(q_new, q_goal, robot, obs, env, interp_steps)
            if node_cnt >= max_nodes
                warning('EST: max_nodes 때문에 goal 노드를 추가하지 못했습니다.');
                break;
            end
            node_cnt          = node_cnt + 1;
            nodes(node_cnt,:) = q_goal;
            parent(node_cnt)  = node_cnt - 1;
            goal_idx          = node_cnt;
            disp("EST: goal reached at iteration " + it);
            break;
        end
    end
end

nodes  = nodes(1:node_cnt,:);
parent = parent(1:node_cnt);

if goal_idx == -1
    warning('EST: 목표에 도달하지 못했습니다.');
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

% EST 트리 
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

title('EST');
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
