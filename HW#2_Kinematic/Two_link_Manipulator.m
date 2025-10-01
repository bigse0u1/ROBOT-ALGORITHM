clear; clc; clear all;

theta1 = 0; theta2 = 0;     % 관절(1, 2) 초기 각도
L1 = 8; L2 = 8;             % 링크(1, 2) 길이

figure; grid on; axis equal;

h = gcf;
set(h, 'WindowKeyPressFcn', @keyPress);

keyPress([], struct('Key','none')); % 한번 그리기

function keyPress(~, event)
    persistent theta1 theta2 L1 L2  % persistent가 입력값 계속 기억, 초l기화 x

    if isempty(theta1)
        theta1 = 0; theta2 = 0;
        L1 = 8; L2 = 8;
    end

    step = 5;                                 % 키 1회당 5도
    key = event.Key;
    switch key
        case 'w', theta1 = theta1 + step;     % w 누르면 theta1 +5도 회전
        case 's', theta1 = theta1 - step;     % s 누르면 theta1 -5도 회전
        case 'e', theta2 = theta2 + step;     % e 누르면 theta2 +5도 회전
        case 'd', theta2 = theta2 - step;     % d 누르면 theta2 -5도 회전
        case 'none'
        otherwise, return;
    end

    DH = @(theta, d, a, alpha)[ ...
        cosd(theta) -sind(theta)*cosd(alpha)  sind(theta)*sind(alpha)  a*cosd(theta); ...
        sind(theta)  cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha)  a*sind(theta); ...
        0            sind(alpha)              cosd(alpha)               d; ...
        0            0                        0                         1];

    T01 = DH(theta1, 0, L1, 0);    
    T12 = DH(theta2, 0, L2, 0);   
    T02 = T01 * T12;               

    p0 = [0;0;0];                  % 각 점 좌표
    p1 = T01(1:3,4);
    p2 = T02(1:3,4);


    clf; hold on; grid on;
    plot3([p0(1) p1(1) p2(1)], [p0(2) p1(2) p2(2)], [0 0 0], 'o-', 'LineWidth', 2); % 링크와 관절 그리지

    R = L1 + L2 + 2;   
    xlim([-R R]); ylim([-R R]); zlim([-R R]);
    grid on          

    xlabel('X'); ylabel('Y'); zlabel('Z'); view(3);
    title(sprintf('Two-link manipulator'));             % 제목

    hold off; drawnow;
end