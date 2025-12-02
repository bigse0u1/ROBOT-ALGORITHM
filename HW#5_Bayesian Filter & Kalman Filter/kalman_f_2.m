clc; clear all; close all;

mu = [5; 5];                    
P  = [50 0; 0 50];              

dt = 1;
u = [1; 1];                     
F = eye(2);                     
B = eye(2);                     
Q = [0.5 0; 0 0.5];             
R = [2.0 0; 0 2.0];             

sensor_locs = [15, 15;
               35, 35;
               81, 81];

true_pos = [5; 5];              
MAX_T = 100;
N = 100;                        

f = figure;
f.Position = [100 100 900 800];

subplot(2, 2, 1);
h_y_pdf = plot(zeros(1, N), 1:N, 'b-', 'LineWidth', 2);
set(gca, 'YDir', 'reverse');    
ylim([1, N]); xlim([0, 0.4]);   
grid on;

subplot(2, 2, 2);
hold on;
plot(sensor_locs(:,2), sensor_locs(:,1), 'go', ...
    'MarkerSize', 10, 'MarkerFaceColor', [.8 1 .8], 'LineWidth', 2);
h_robot = plot(true_pos(2), true_pos(1), 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
h_est_dot = plot(mu(2), mu(1), 'r.', 'MarkerSize', 15);
h_ellip = plot(0, 0, 'r-', 'LineWidth', 2); 
hold off;
axis square; grid on;
set(gca, 'YDir', 'reverse');
xlim([1, N]); ylim([1, N]);

subplot(2, 2, 4);
h_x_pdf = plot(1:N, zeros(1, N), 'r-', 'LineWidth', 2);
ylim([0, 0.4]); xlim([1, N]);
grid on;

for k = 1:MAX_T
    mu = F * mu + B * u;
    P = F * P * F' + Q;
    
    true_pos = true_pos + u + 0.2 * randn(2,1);
    if true_pos(1) > N || true_pos(2) > N, break; end
    
    is_detected = false;
    z = [];
    
    for s_idx = 1:size(sensor_locs, 1)
        s_pos = sensor_locs(s_idx, :)'; 
        dist = norm(true_pos - s_pos);
        if dist < 5
            is_detected = true;
            z = true_pos + sqrt(R(1,1)) * randn(2,1); 
            break; 
        end
    end
    
    if is_detected
        K = P * inv(P + R);
        mu = mu + K * (z - mu);
        P = (eye(2) - K) * P;
        status_color = 'r';
        status_msg = 'DETECTED!';
    else
        status_color = 'k';
        status_msg = 'Moving...';
    end

    draw_fixed_ellipse(h_ellip, mu, P, 3);
    
    set(h_robot, 'XData', true_pos(2), 'YData', true_pos(1));
    set(h_est_dot, 'XData', mu(2), 'YData', mu(1));
    
    pos_axis = 1:N;
    pdf_y = custom_normpdf(pos_axis, mu(1), sqrt(P(1,1))); 
    set(h_y_pdf, 'XData', pdf_y); 
    
    pdf_x = custom_normpdf(pos_axis, mu(2), sqrt(P(2,2)));
    set(h_x_pdf, 'YData', pdf_x);
   
    
    drawnow;
    pause(0.1); 
end

function draw_fixed_ellipse(h_plot, mu, P, n_sigma)
    t = linspace(0, 2*pi, 50);
    
    sigma_y = sqrt(P(1,1));
    sigma_x = sqrt(P(2,2));
    
    x_points = mu(2) + n_sigma * sigma_x * cos(t);
    y_points = mu(1) + n_sigma * sigma_y * sin(t);
    
    set(h_plot, 'XData', x_points, 'YData', y_points);
end

function y = custom_normpdf(x, mu, sigma)
    y = (1 ./ (sqrt(2*pi) .* sigma)) .* exp(-0.5 * ((x - mu) ./ sigma).^2);
end