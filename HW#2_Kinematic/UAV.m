function move_triangle()
    tri = [0  1 -1; 
           0 -1 -1; 
           0  0  0];   %    비행물체

    tri_h = [tri; ones(1,3)];

    T = eye(4);

    fig = figure('Name','3D Triangle Move');    % 그림 그리기
    axis equal; grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim([-10 10]); ylim([-10 10]); zlim([-10 10]);
    view(45,30);

    set(fig,'WindowKeyPressFcn',@keyPress);

    drawTriangle();

    function keyPress(~,evt)
        stepT = 0.5;   % 이동 거리
        stepR = 5;     % 회전 각도
        switch evt.Key

            case 'u' , T = T * transl(0,0, stepT); % 앞으로
            case 'j' , T = T * transl(0,0,-stepT); % 뒤로
            case 'i' , T = T * transl(-stepT,0,0); % 왼쪽
            case 'k' , T = T * transl(stepT,0,0); % 오른쪽
            case 'o' , T = T * transl(0, stepT,0); % 상승
            case 'l' , T = T * transl(0,-stepT,0); % 하강

            case 'w' , T = T * rotx(stepR); % pitch +
            case 's' , T = T * rotx(-stepR); % pitch -
            case 'a' , T = T * roty(stepR); % yaw +
            case 'd' , T = T * roty(-stepR); % yaw -
            case 'q' , T = T * rotz(stepR); % roll +
            case 'e' , T = T * rotz(-stepR);  % roll -
        end
        drawTriangle();
    end

    function drawTriangle()
        clf; hold on; grid on;
        axis equal;
        xlim([-10 10]); ylim([-10 10]); zlim([-10 10]);
        xlabel('X'); ylabel('Y'); zlabel('Z'); view(45,30);

        tri_t = T * tri_h;

        fill3(tri_t(1,:), tri_t(2,:), tri_t(3,:), 'r');
    end

    function T = transl(x,y,z)
        T = eye(4); T(1:3,4) = [x;y;z];
    end
    function R = rotx(a)
        ca=cosd(a); sa=sind(a);
        R=[1 0 0 0;0 ca -sa 0;0 sa ca 0;0 0 0 1];
    end
    function R = roty(a)
        ca=cosd(a); sa=sind(a);
        R=[ca 0 sa 0;0 1 0 0;-sa 0 ca 0;0 0 0 1];
    end
    function R = rotz(a)
        ca=cosd(a); sa=sind(a);
        R=[ca -sa 0 0;sa ca 0 0;0 0 1 0;0 0 0 1];
    end
    xlabel('X'); ylabel('Y'); zlabel('Z'); 
    view(45,30);
    title(sprintf('UAV'));          % 제목

    hold off; drawnow;
end