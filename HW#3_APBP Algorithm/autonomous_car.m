clc; clear all; close all;

attraction = 10;    % 인력 계수 
epsilon    = 5;     % 목표 근처에서 인력 함수를 바꾸는 거리
trans_step = 0.1;   % 이동 보폭
rot_step   = 0.05;  % 회전 보폭

start_q = [10, 20, pi/2]; % 로봇 초기

goal_points = [ 80, 80; 70, 70; 90, 70 ];   % 목표 위치

robot_points = [ 0, 10; -10, 0; 10, 0 ];    % 로봇

robot_q  = start_q; % 로봇 시작 초기화
max_iter = 2000;

figure; 
plot(goal_points(:,1), goal_points(:,2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 15); hold on;
xlim([0 100]); ylim([0 100]); axis manual 
grid on;
daspect([1 1 1]);
title('Autonomous Car');
xlabel('X'); ylabel('Y');

robot_current = get_global_points(robot_q, robot_points);
h_robot = patch(robot_current(:,1), robot_current(:,2), 'b', 'FaceAlpha', 0.5);
drawnow;

for i = 1:max_iter
    robot_current = get_global_points(robot_q, robot_points);
    
    dist1 = norm(robot_current(1,:) - goal_points(1,:));
    dist2 = norm(robot_current(2,:) - goal_points(2,:));
    dist3 = norm(robot_current(3,:) - goal_points(3,:));
    
    if dist1 < 1.5 && dist2 < 1.5 && dist3 < 1.5    % 목표에 충분히 가까워지면 종료
        fprintf('목표 자세에 도달했습니다!\n');
        break;
    end
    
    F_Q = zeros(3, 1); % 힘
    
    for k = 1:3
        p_global = robot_current(k, :); % 로봇의 k번째 제어점
        p_goal   = goal_points(k, :);           % k번째 목표점
        
        dist_to_goal = norm(p_global - p_goal); % 인력
        if dist_to_goal > epsilon
            F_att = -(epsilon * attraction * (p_global - p_goal)) / dist_to_goal;
        else
            F_att = -attraction * (p_global - p_goal);
        end
        
        F_W = F_att';
        
        p_local = robot_points(k, :);   % 자코비안
        J = get_jacobian(robot_q(3), p_local);
        
        F_Q = F_Q + J' * F_W;
    end
    
    force_vec = F_Q(1:2);
    torque    = F_Q(3);
    
    if norm(force_vec) > 1e-6
        robot_q(1:2) = robot_q(1:2) + trans_step * (force_vec / norm(force_vec))';
    end
    
    if abs(torque) > 1e-6
        robot_q(3) = robot_q(3) + rot_step * (torque / abs(torque));
    end
    
    updated_points = get_global_points(robot_q, robot_points);
    set(h_robot, 'Vertices', updated_points);
    drawnow;
end

if i == max_iter
    fprintf('최대 반복에 도달했습니다.\n');
end

function global_points = get_global_points(q, local_points)
    x = q(1); y = q(2); theta = q(3);
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    global_points = (R * local_points' + [x; y])';
end

function J = get_jacobian(theta, p_local)
    ax = p_local(1); ay = p_local(2);
    J = [1, 0, -ax*sin(theta) - ay*cos(theta);
         0, 1,  ax*cos(theta) - ay*sin(theta)];
end