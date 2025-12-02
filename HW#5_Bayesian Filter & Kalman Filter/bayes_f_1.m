clc; clear all; close all;

N = 200;
bel = ones(N, 1);
bel = bel / sum(bel); 

M_plus = zeros(N, N);
for i = 1:N
    if i + 1 <= N, M_plus(i + 1, i) = 1/3; end
    if i + 2 <= N, M_plus(i + 2, i) = 1/3; end
    if i + 3 <= N, M_plus(i + 3, i) = 1/3; end
end

sensor_pos = [41 81 141];
sample     = 1:N;

sensor_model_base = measure_1D(sensor_pos, N);

sensor_model = zeros(N, N);
for z = 1:N
    if any(z == sensor_pos)
        sensor_model(z, :) = sensor_model_base;
    else
        sensor_model(z, :) = ones(1, N);
        sensor_model(z, :) = sensor_model(z, :) / sum(sensor_model(z, :));
    end
end

x = 1;
MAX_T = 100;

max_bel_guess = 0.4;

f = figure;
f.Position = [100 100 1200 700];

subplot(2,1,1);
bh = plot(sample, bel, "b", "LineWidth", 2);
axis([1, N, 0, max_bel_guess]);
grid on;

subplot(2,1,2);
plot(sensor_pos, ones(size(sensor_pos)), 'go', ...
    "MarkerSize", 10, "MarkerFaceColor", [.5 .9 .5]);
hold on;
rh = plot(x, 1, 'bs', ...
    "MarkerSize", 15, "MarkerFaceColor", [0.5 0.5 1]);
hold off;
axis([1, N, 0, 2]);
grid on;

for k = 1:MAX_T
    subplot(2,1,1);
    set(bh, 'YData', bel); 
    subplot(2,1,2);
    set(rh, 'XData', x, 'YData', 1);  

    drawnow;
    pause(0.05);  

    z = x;

    bel = bayes_filter(bel, z, sensor_model, M_plus);

    x = x + 2;
    if x >= N
        x = N;
        subplot(2,1,2);
        set(rh, 'XData', x, 'YData', 1);
        drawnow;
        break;
    end
end


function bel = bayes_filter(bel, z, sensor_model, M)
    bel = M * bel;
    bel = sensor_model(z, :)' .* bel;
    bel = bel / (sum(bel) + eps);
end

function gf = Gaussian_1D(sigma, size_k)
    half_len = fix((size_k + 1) / 2);
    gf1 = zeros(1, half_len);
    for x = 0:(half_len - 1)
        gf1(x + 1) = 1 / (sqrt(2*pi)*sigma^2) * exp(-(x^2)/(2*sigma^2));
    end
    gf2 = gf1; gf2(1) = []; gf2 = fliplr(gf2);
    gf = [gf2 gf1];
end

function sensor_model = measure_1D(sensor_pos, N)
    sensor_model_single = Gaussian_1D(1.5, 12);
    sensor_model = zeros(1, N);
    for j = 1:length(sensor_pos)
        sp = sensor_pos(j);
        lb = sp - floor(length(sensor_model_single)/2);
        for i = 1:length(sensor_model_single)
            idx = lb + i - 1;
            if idx >= 1 && idx <= N
                sensor_model(idx) = max(sensor_model(idx), sensor_model_single(i));
            end
        end
    end
    sensor_model = sensor_model / (sum(sensor_model) + eps);
end
