clc; clear all; close all;

N = 100;
bel = ones(N, N);
bel = bel / sum(bel(:));

motion_kernel = [0   0     0; 
                 0   0.05  0.2; 
                 0   0.2   0.55];
motion_kernel = motion_kernel / sum(motion_kernel(:));

sensor_locs = [15, 15;
               35, 35;
               81, 81];

likelihood_all_sensors = zeros(N, N);
for i = 1:size(sensor_locs, 1)
    g = get_gaussian_2d(N, sensor_locs(i, :), 3.0);
    likelihood_all_sensors = max(likelihood_all_sensors, g);
end
likelihood_all_sensors = likelihood_all_sensors + 1e-9;

rx = 5; ry = 5;
MAX_T = 100;
f = figure;
f.Position = [100 100 900 800];

subplot(2, 2, 1);
h_y = plot(zeros(1, N), 1:N, 'b-', 'LineWidth', 2);
set(gca, 'YDir', 'reverse');
ylim([1, N]); xlim([0, 0.4]);
grid on;

subplot(2, 2, 2);
hold on;
plot(sensor_locs(:,2), sensor_locs(:,1), 'go', ...
    'MarkerSize', 10, 'MarkerFaceColor', [.8 1 .8], 'LineWidth', 2);
h_robot = plot(ry, rx, 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 10);

h_est = plot(N/2, N/2, 'ro', 'LineWidth', 2, 'MarkerSize', 50); 
h_est_center = plot(N/2, N/2, 'r.', 'MarkerSize', 20);          

hold off;
axis square; grid on;
set(gca, 'YDir', 'reverse');
xlim([1, N]); ylim([1, N]);

subplot(2, 2, 4);
h_x = plot(1:N, zeros(1, N), 'r-', 'LineWidth', 2);
ylim([0, 0.4]);
grid on;

for k = 1:MAX_T
    rx = rx + 1; ry = ry + 1;
    if rx > N || ry > N, break; end
    
    is_detected = false;
    for s = 1:size(sensor_locs,1)
        dist = hypot(rx - sensor_locs(s,1), ry - sensor_locs(s,2));
        if dist < 5
            is_detected = true;
            break;
        end
    end
    
    if is_detected
        msr_likelihood = likelihood_all_sensors;
    else
        msr_likelihood = ones(N, N);
    end
    
    bel = conv2(bel, motion_kernel, 'same');
    bel = bel .* msr_likelihood;
    bel = bel / sum(bel(:) + eps);
    
    [max_val, max_idx] = max(bel(:));
    [est_r, est_c] = ind2sub(size(bel), max_idx);
    
    sensitivity = 2.0; 
    est_size = sensitivity / (max_val + 1e-4); 
    est_size = min(150, max(5, est_size)); 
    
    set(h_est, 'XData', est_c, 'YData', est_r, 'MarkerSize', est_size);
    set(h_est_center, 'XData', est_c, 'YData', est_r);
    
    bel_x = sum(bel, 1);
    bel_y = sum(bel, 2)';
    
    set(h_robot, 'XData', ry, 'YData', rx);
    set(h_x, 'YData', bel_x);
    set(h_y, 'XData', bel_y);
    
    drawnow;
    pause(0.1); 
end

function g_map = get_gaussian_2d(N, center, sigma)
    [X, Y] = meshgrid(1:N, 1:N);
    exponent = -((X - center(2)).^2 + (Y - center(1)).^2)/(2*sigma^2);
    g_map = exp(exponent);
end