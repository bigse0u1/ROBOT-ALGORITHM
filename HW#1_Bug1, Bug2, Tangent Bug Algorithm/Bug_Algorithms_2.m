clc; clear all; close all;

start = [0, 0]; goal = [10, 10];    % 시작점, 도착점

box = [ 4, 2, 2, 4;   % 첫 번째 장애물
        8, 5, 2, 4 ]; % 두 번째 장애물

p = start; step = 0.05; onB = false; reached = false;   % onB: 테두리 따라가기, reached: 도착
Hit = [];               % 장애물에 부딪친 장소
bSteps = 0; 

figure;
set(gcf, 'Color', 'k'); set(gca, 'Color', 'w'); set(gca, 'GridColor', 'k'); % 표 그리기
grid on; hold on; axis equal;
title('Bug Algorithm_2'); xlabel('X'); ylabel('Y'); % 제목, 가로세로축

plot(start(1), start(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');       % 시작점
plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');         % 도착점

for i = 1:size(box,1)
    rectangle('Position', box(i,:), 'EdgeColor', 'k', 'FaceColor', [.8 .8 .8]); % 장애물
end

rh = plot(p(1), p(2), 'b.', 'MarkerSize', 20);      % 로봇 위치
ph = plot(p(1), p(2), 'c.');                        % 이동 경로
hh = plot(NaN, NaN, 'k.', 'MarkerSize', 30);        % 장애물이랑 부딪친 장소 표시

r_45   = -pi / 4;                                   % 우측 45도 회전
R_right45 = [cos(r_45), -sin(r_45); sin(r_45), cos(r_45)];
l_90   =  pi / 2;                                   % 좌측 90도 회전
R_left90  = [cos(l_90), -sin(l_90); sin(l_90), cos(l_90)];

line_vec = goal - start;                            % 출발점→도착점 벡터

isInsideAny = @(pt) any( pt(1) >= box(:,1) & pt(1) <= box(:,1)+box(:,3) & pt(2) >= box(:,2) & pt(2) <= box(:,2)+box(:,4) );

while ~reached  % goal에 도착할 때까지
    if norm(p - goal) < step    % 남은 거리가 step보다 작으면 도착
        reached = true;
        break;
    end
    
    if ~onB % 직진 모드
        d  = (goal - p) / norm(goal - p);   % 목표 방향
        np = p + step * d;                  % 다음 위치
        
        inside = isInsideAny(np);            % 장애물과라도 충돌?

        if inside               % 만약 장애물과 출돌했다면
            onB   = true;       % 테두리 따라가기 on
            Hit   = p;          % 부딪친 지점 저장
            set(hh, 'XData', Hit(1), 'YData', Hit(2));
            bSteps = 0;         % 장애물 따라 움직인 걸음
            d = d * R_right45;  % 우측 45도 회전
        else
            p = np;             % 출돌하지 않았으면 직진
        end
        
    else 
        bSteps = bSteps + 1;
        
        current_vec = p - start; % % 출발점 기준 현재 백터
        cross_z = current_vec(1) * line_vec(2) - current_vec(2) * line_vec(1); 
        
        if bSteps > 10 && abs(cross_z) < 0.1 && norm(p - goal) < norm(Hit - goal) % 다 만족하면
            onB = false; % 직진 모드로 전환
            Hit = [];
            bSteps = 0;
            continue; % 다음 루프부터 직진
        end
        
        l_sensor   = d * R_left90;           % 왼쪽 90도 회전
        left_check = p + step * l_sensor;
        left_blocked = isInsideAny(left_check); % 왼쪽에 벽이 있는가?
        
        if ~left_blocked % 왼쪽이 비었으면 왼쪽으로 붙어서 전진
            d = l_sensor; 
        else
            front_check  = p + step * d;
            front_blocked = isInsideAny(front_check);
            if front_blocked
                d = d * R_right45; % 정면 막혔으면 우측으로 살짝 회전
            end
        end
        
        p = p + step * d;
    end
    
    set(rh, 'XData', p(1), 'YData', p(2));
    pause(0.01);
end