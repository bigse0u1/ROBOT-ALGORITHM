clear; clc; clear all;

theta1 = 0; theta2 = 0; theta3 = 0; theta4 = 0;
L1 = 8; L2 = 8; L3 = 6; L4 = 5;

figure; grid on; axis equal;

h = gcf;
set(h, 'WindowKeyPressFcn', @keyPress);

keyPress([], struct('Key','none'));

function keyPress(~, event)
    persistent theta1 theta2 theta3 theta4 L1 L2 L3 L4

    if isempty(theta1)
        theta1 = 0; theta2 = 0; theta3 = 0; theta4 = 0;
        L1 = 8; L2 = 8; L3 = 6; L4 = 5;
    end

    step = 5;                               % 키 1회당 5도
    key = event.Key;
    switch key
        case 'w', theta1 = theta1 + step;   % 1번: z축 회전
        case 's', theta1 = theta1 - step;

        case 'e', theta2 = theta2 + step;   % 2번: y축 회전
        case 'd', theta2 = theta2 - step;

        case 'r', theta3 = theta3 + step;   % 3번: y축 회전
        case 'f', theta3 = theta3 - step;

        case 't', theta4 = theta4 + step;   % 4번: y축 회전
        case 'g', theta4 = theta4 - step;

        case 'none' % 최초 1회 그리기용
        otherwise
            return;
    end

    Rz = @(th) [ cosd(th) -sind(th) 0;
                 sind(th)  cosd(th) 0;
                 0         0        1 ];
    Ry = @(th) [ cosd(th)  0  sind(th);
                 0         1  0;
                -sind(th)  0  cosd(th) ];

    p0 = [0;0;0];         % 베이스
    p1 = [0;0;L1];        % 1번 링크: 항상 +Z로

    R0 = Rz(theta1);                 % 1번 관절
    R1 = R0 * Ry(theta2);            % 2번 관절
    R2 = R1 * Ry(theta3);            % 3번 관절
    R3 = R2 * Ry(theta4);            % 4번 관절

    p2 = p1 + R1 * [L2;0;0];
    p3 = p2 + R2 * [L3;0;0];
    p4 = p3 + R3 * [L4;0;0];

    clf; hold on; grid on; axis equal;
    R = L1 + L2 + L3 + L4 + 2;
    xlim([-R R]); ylim([-R R]); zlim([0 R]);  
    xlabel('X'); ylabel('Y'); zlabel('Z'); view(45,30);

    plot3([p0(1) p1(1)], [p0(2) p1(2)], [p0(3) p1(3)], '-o', 'LineWidth', 3);

    plot3([p1(1) p2(1) p3(1) p4(1)], ...
          [p1(2) p2(2) p3(2) p4(2)], ...
          [p1(3) p2(3) p3(3) p4(3)], '-o', 'LineWidth', 3);

    xlabel('X'); ylabel('Y'); zlabel('Z'); view(3);
    title(sprintf('Manipulator X'));             % 제목

    hold off; drawnow;
end