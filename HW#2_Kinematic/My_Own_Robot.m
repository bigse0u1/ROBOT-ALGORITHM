clear; clc; clear all;

figure; grid on; axis equal;
h = gcf;
set(h, 'WindowKeyPressFcn', @keyPress);

keyPress([], struct('Key','none'));

function keyPress(~, event)
    persistent thA thB thC L1 L2 L3 L4 L5 L1c L2c sep
    if isempty(thA)
        thA = [180 0 0];     % 그룹 A 초기자세 (로봇 1·2)
        thB = [180 0 0];     % 그룹 B 초기자세 (로봇 3·4)
        thC = [180 0];       % ★ 추가: 2축 로봇(θ1, θ2)

        L1 = 8; L2 = 8; L3 = 6; 
        L4 = 10; L5 = 11;     % 일부 로봇에 다른 L1을 주려던 기존 변형 유지

        L1c = 6; L2c = 5;

        sep = 4;
    end

    step = 5; 
    switch event.Key
        % ---- 그룹 A (로봇 1·2) ----
        case 'w', thA(1) = thA(1) + step;
        case 's', thA(1) = thA(1) - step;
        case 'e', thA(2) = thA(2) + step;
        case 'd', thA(2) = thA(2) - step;
        case 'r', thA(3) = thA(3) + step;
        case 'f', thA(3) = thA(3) - step;

        % ---- 그룹 B (로봇 3·4) ----
        case 'i', thB(1) = thB(1) + step;
        case 'k', thB(1) = thB(1) - step;
        case 'o', thB(2) = thB(2) + step;
        case 'l', thB(2) = thB(2) - step;
        case 'p', thB(3) = thB(3) + step;
        case 'semicolon', thB(3) = thB(3) - step;

        % ---- ★ 2축 로봇(별도) ----
        case 't', thC(1) = thC(1) + step;   
        case 'g', thC(1) = thC(1) - step;  
        case 'y', thC(2) = thC(2) + step;  
        case 'h', thC(2) = thC(2) - step;   

        case 'escape', close(gcf); return;
        case 'none'   
        otherwise, return;
    end

    thA = max(min(thA, [180 180 180]), [-180 -180 -180]); % 관절 제한
    thB = max(min(thB, [180 180 180]), [-180 -180 -180]);
    thC = max(min(thC, [180 180]),     [-180 -180]);

    DH = @(theta, d, a, alpha)[ ...
        cosd(theta) -sind(theta)*cosd(alpha)  sind(theta)*sind(alpha)  a*cosd(theta); ...
        sind(theta)  cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha)  a*sind(theta); ...
        0            sind(alpha)              cosd(alpha)               d; ...
        0            0                        0                         1];
    Trans = @(x,y,z)[1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
    RotY  = @(a)[cosd(a) 0 sind(a) 0; 0 1 0 0; -sind(a) 0 cosd(a) 0; 0 0 0 1];

    tilt = RotY(90);                
    base1 = Trans(0*sep, 0, 0) * tilt;   % 로봇 1
    base2 = Trans(1*sep, 0, 0) * tilt;   % 로봇 2
    base3 = Trans(2*sep, 0, 0) * tilt;   % 로봇 3
    base4 = Trans(3*sep, 0, 0) * tilt;   % 로봇 4

    baseC = Trans(-4, 0, -4) * tilt;

    [p0_1,p1_1,p2_1,p3_1] = fk3(base1, thA, L1, L2, L3, DH);
    [p0_2,p1_2,p2_2,p3_2] = fk3(base2, thA, L5, L2, L3, DH);

    [p0_3,p1_3,p2_3,p3_3] = fk3(base3, thB, L4, L2, L3, DH);
    [p0_4,p1_4,p2_4,p3_4] = fk3(base4, thB, L1, L2, L3, DH);

    [pc0,pc1,pc2] = fk2(baseC, thC, L1c, L2c, DH);

    clf; hold on; grid on; axis equal;

    square = [ 0 0 0; 12 0 0; 12 0 -15; 0 0 -15 ];
    fill3(square(:,1), square(:,2), square(:,3), 'b');

    square2 = [ -4 0 -4; 0 0 0; 0 0 -15 ];
    fill3(square2(:,1), square2(:,2), square2(:,3), 'b');

    drawRobot3(p0_1,p1_1,p2_1,p3_1, [0 0.45 0.74], '1');        % 그룹 A (파란)
    drawRobot3(p0_2,p1_2,p2_2,p3_2, [0 0.45 0.74], '2');

    drawRobot3(p0_3,p1_3,p2_3,p3_3, [0.85 0.33 0.10], '3');     % 그룹 B (주황)
    drawRobot3(p0_4,p1_4,p2_4,p3_4, [0.85 0.33 0.10], '4');

    drawRobot2(pc0,pc1,pc2, [0.20 0.60 0.20], 'C');             % 2축 로봇 (초록)

    xlabel('X'); ylabel('Y'); zlabel('Z'); view(45,30);
    title('My Own Robot');

    hold off; drawnow;

    function [p0,p1,p2,p3] = fk3(base, th, L1_, L2_, L3_, DH_)
        T01 = base * DH_(th(1), 0, L1_, 0);
        T02 = T01  * DH_(th(2), 0, L2_, 0);
        T03 = T02  * DH_(th(3), 0, L3_, 0);
        p0 = base(1:3,4);
        p1 = T01(1:3,4);
        p2 = T02(1:3,4);
        p3 = T03(1:3,4);
    end

    function [p0,p1,p2] = fk2(base, th, L1_, L2_, DH_)
        T01 = base * DH_(th(1), 0, L1_, 0);
        T02 = T01  * DH_(th(2), 0, L2_, 0);
        p0 = base(1:3,4);
        p1 = T01(1:3,4);
        p2 = T02(1:3,4);
    end

    function drawRobot3(p0,p1,p2,p3, col, lbl)
        plot3([p0(1) p1(1) p2(1) p3(1)], ...
              [p0(2) p1(2) p2(2) p3(2)], ...
              [p0(3) p1(3) p2(3) p3(3)], ...
              'o-','LineWidth',2,'Color',col);
        text(p0(1),p0(2),p0(3), ['  R' lbl], 'Color',col, 'FontWeight','bold');
    end

    function drawRobot2(p0,p1,p2, col, lbl)
        plot3([p0(1) p1(1) p2(1)], ...
              [p0(2) p1(2) p2(2)], ...
              [p0(3) p1(3) p2(3)], ...
              'o-','LineWidth',2,'Color',col);
        text(p0(1),p0(2),p0(3), ['  R' lbl], 'Color',col, 'FontWeight','bold');
    end
end
