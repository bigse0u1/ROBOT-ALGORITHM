clc; clear all; close all;

N = 200;
mu = 1;
P  = 10;

u = 2;
Q = 0.5;
R = 2.0;

sensor_pos = [41 81 141];       

x_true = 1;
MAX_T = 100;

f = figure; f.Position = [100 100 1000 600];

subplot(2,1,1);
x_axis = 1:0.5:N;               
h_bel = plot(x_axis, zeros(size(x_axis)), 'b', 'LineWidth', 2);
axis([1, N, 0, 0.4]);           
grid on; 

subplot(2,1,2);
hold on;
plot(sensor_pos, ones(size(sensor_pos)), 'go', ...
    "MarkerSize", 10, "MarkerFaceColor", [.5 .9 .5]);

h_robot = plot(x_true, 1, 'bs', "MarkerSize", 15, "MarkerFaceColor", [0.5 0.5 1]);

h_est = plot(mu, 1, 'ro', "MarkerSize", 8, "MarkerFaceColor", 'r');

hold off;
set(gca, 'YTick', []);
axis([1, N, 0.5, 1.5]);
grid on;

for k = 1:MAX_T
    mu = mu + u;
    P  = P + Q;
    
    x_true = x_true + u + 0.1 * randn;
    
    z = NaN; 
    for s_idx = 1:length(sensor_pos)
        if abs(x_true - sensor_pos(s_idx)) < 3
            z = x_true + sqrt(R) * randn;
            break;
        end
    end
    
    if ~isnan(z)
        K = P / (P + R); 
        mu = mu + K * (z - mu);
        P = (1 - K) * P;
        
        status_msg = 'SENSOR DETECTED! (Correction)';
        status_color = 'r';
    else
        status_msg = 'Moving... (Prediction)';
        status_color = 'k';
    end
        
    sigma = sqrt(P);
    y_gauss = (1 / (sqrt(2*pi)*sigma)) * exp(-((x_axis - mu).^2) / (2*P));
    set(h_bel, 'YData', y_gauss);
    
    set(h_robot, 'XData', x_true); 
    set(h_est, 'XData', mu);       
    
    if x_true >= N, break; end
    
    drawnow;
    pause(0.05);
end