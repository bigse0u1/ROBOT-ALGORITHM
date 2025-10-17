clc; clear all; close all;

attraction = 10;     % 인력 계수
epsilon    = 5;      % 목표 근처에서 인력 함수를 바꾸는 거리
trans_step = 0.1;    % 이동 보폭
rot_step   = 0.01;   % 회전 보폭

obs_radius = 30;     % 구 반지름
eta_rep    = 250;    % 척력 세기
d0         = 45;     % 척력 유효 범위 

start_q = [10, 10, 10, 0, pi, 0];     
goal_points = [ 80, 80, 70;
                70, 90, 60;
                90, 90, 60 ];
robot_points = [ 20, 20, 10;
                 10, 20,  0;
                 30, 30,  0 ];

robot_q  = start_q;
max_iter = 3000;

start_pos   = robot_q(1:3);
goal_center = mean(goal_points,1);
obs_center  = (start_pos + goal_center)/2;      

figure; hold on;
plot3(goal_points(:,1), goal_points(:,2), goal_points(:,3), ...
      'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5);

[sx,sy,sz] = sphere(40); % 구(장애물)
surf(sx*obs_radius + obs_center(1), sy*obs_radius + obs_center(2), sz*obs_radius + obs_center(3), 'FaceAlpha',0.15,'EdgeColor','none','FaceColor',[0 0.6 0.2]);

hAx = gca;
xlim(hAx, [-20 100]); ylim(hAx, [-20 100]); zlim(hAx, [-20 100]); axis(hAx,'manual');
set(hAx,'XLimMode','manual','YLimMode','manual','ZLimMode','manual');
grid(hAx,'on'); daspect(hAx,[1 1 1]); pbaspect(hAx,[1 1 1]); axis(hAx,'vis3d');
view(3);
title('My Own Robot - Obstacle Avoidance');
xlabel('X'); ylabel('Y'); zlabel('Z');

current_points_global = get_global_points_3d(robot_q, robot_points);
h_robot_body = patch('Vertices', current_points_global, 'Faces', [1 2 3], ...
                     'FaceColor','b','FaceAlpha',0.5);
drawnow;

for i = 1:max_iter
    current_points_global = get_global_points_3d(robot_q, robot_points);
    
    d1 = norm(current_points_global(1,:) - goal_points(1,:));
    d2 = norm(current_points_global(2,:) - goal_points(2,:));
    d3 = norm(current_points_global(3,:) - goal_points(3,:));
    if d1 < 2 && d2 < 2 && d3 < 2
        break;
    end
    
    F_Q = zeros(6,1);  
    for k = 1:3
        p_global = current_points_global(k, :)'; 
        p_goal   = goal_points(k, :)';           
        
        e = p_global - p_goal;
        d = norm(e);
        if d > epsilon
            F_att = -(epsilon * attraction) * e / max(d,1e-12);
        else
            F_att = -attraction * e;
        end

        v  = p_global - obs_center(:);      % 구 중심 -> 점
        R  = norm(v);                       % 중심까지 거리
        if R < 1e-12, v = [1;0;0]; R = 1e-12; end
        s  = R - obs_radius;                % 구 표면까지의 거리 (clearance)
        F_rep = [0;0;0];
        if s <= 0
            d_hat = v / R;
            s_eff = 1e-3;
            grad  = (1/s_eff - 1/d0) * (1/(s_eff^2)) * d_hat;
            F_rep = eta_rep * grad;
        elseif s < d0
            d_hat = v / R;
            grad  = (1/s - 1/d0) * (1/(s^2)) * d_hat;
            F_rep = eta_rep * grad;
        end
        
        F_W = F_att + F_rep;   
        
        J = get_jacobian_3d(robot_q(4:6), robot_points(k,:));
        
        F_Q = F_Q + J' * F_W;
    end
    
    force_vec  = F_Q(1:3);
    torque_vec = F_Q(4:6);
    
    if norm(force_vec)  > 1e-6
        robot_q(1:3) = robot_q(1:3) + trans_step * (force_vec / norm(force_vec))';
    end
    if norm(torque_vec) > 1e-6
        robot_q(4:6) = robot_q(4:6) +  rot_step * (torque_vec / norm(torque_vec))';
    end
    
    updated_points = get_global_points_3d(robot_q, robot_points);
    set(h_robot_body, 'Vertices', updated_points);
    drawnow;
end

function global_points = get_global_points_3d(q, local_points)
    x = q(1); y = q(2); z = q(3);
    roll = q(4); pitch = q(5); yaw = q(6);
    Rx = [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)];
    Ry = [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)];
    Rz = [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1];
    R  = Rz * Ry * Rx;
    global_points = (R * local_points' + [x; y; z])';
end

function J = get_jacobian_3d(angles, p_local)
    roll = angles(1); pitch = angles(2); yaw = angles(3);
    Rx = [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)];
    Ry = [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)];
    Rz = [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1];
    R  = Rz * Ry * Rx;
    
    pg = R * p_local(:); 
    S = [   0   -pg(3)  pg(2);
          pg(3)   0    -pg(1);
         -pg(2)  pg(1)   0  ];
    
    J = [eye(3), -S];
end
