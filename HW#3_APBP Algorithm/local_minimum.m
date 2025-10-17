clc; clear all; close all;

attraction = 10;     % 인력 계수
repulsion  = 200;   % 척력 계수
epsilon    = 15;    % 목표 근처에서 인력 함수를 바꾸는 거리
delta      = 20;    % 장애물 영향 반경
step_size  = 0.1;   % 이동 보폭
safety_margin = 5;  % 최소거리

start_p = [0, 50];    % 시작점
goal_p  = [100, 50];  % 도착점

obs_x = [45 50 45];
obs_y = [40 50 60];
obstacles = [obs_x', obs_y'];   % 장애물

robot_pos  = start_p;
max_iter   = 500;

figure;
plot(start_p(1), start_p(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10); hold on;
plot(goal_p(1),  goal_p(2),  'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
plot(obstacles(:,1), obstacles(:,2), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
h_robot = plot(robot_pos(1), robot_pos(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);

xlim([0 100]); ylim([0 100]); axis manual 
grid on;
daspect([1 1 1]);
title('Local Minimum');
xlabel('X'); ylabel('Y');
drawnow;

for i = 1:max_iter
    dist_to_goal = norm(robot_pos - goal_p);

    if dist_to_goal < 1
        fprintf('목표 지점에 도달했습니다!\n');
        break;
    end

    if dist_to_goal > epsilon
        F_att = -(epsilon * attraction) * (robot_pos - goal_p) / dist_to_goal;
    else
        F_att = -attraction * (robot_pos - goal_p);
    end

    F_rep_total = [0, 0];
    is_too_close = false;

    for j = 1:size(obstacles, 1)
        obs_point   = obstacles(j, :);
        dist_to_obs = norm(robot_pos - obs_point);

        if dist_to_obs <= safety_margin
            fprintf('안전거리(%.1f) 침범! 로봇이 정지합니다.\n', safety_margin);
            is_too_close = true;
            break;
        end

        if dist_to_obs < delta
            grad = (robot_pos - obs_point) / dist_to_obs; 
            rep_force = repulsion * (1/dist_to_obs - 1/delta) * (1/dist_to_obs^2) .* grad;
            F_rep_total = F_rep_total + rep_force;
        end
    end

    F_total = F_att + F_rep_total;

    if ~is_too_close
        denom = max(norm(F_total), 1e-12);
        robot_pos = robot_pos + step_size * (F_total / denom);
    end

    set(h_robot, 'XData', robot_pos(1), 'YData', robot_pos(2));
    drawnow;
en