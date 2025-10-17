clc; clear all; close all;

attraction = 10;     
epsilon    = 5;     
trans_step = 0.1;   
rot_step   = 0.01;   

start_q = [10, 10, 10, 0, pi, 0];     % [x, y, z, roll, pitch, yaw]
goal_points = [ 80, 80, 70; 70, 90, 60; 90, 90, 60 ];
robot_points = [ 20, 20, 10; 10, 20,  0; 30, 30,  0 ];

robot_q  = start_q;
max_iter = 3000;

figure; hold on;
plot3(goal_points(:,1), goal_points(:,2), goal_points(:,3), ...
      'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5);

hAx = gca;
xlim(hAx, [-20 100]); ylim(hAx, [-20 100]); zlim(hAx, [-20 100]); axis(hAx,'manual');
set(hAx,'XLimMode','manual','YLimMode','manual','ZLimMode','manual');
grid(hAx,'on'); daspect(hAx,[1 1 1]); pbaspect(hAx,[1 1 1]); axis(hAx,'vis3d');
view(3);
title('UAV - APBP (Textbook Jacobian)');
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
        
        F_W = F_att;          
        
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
