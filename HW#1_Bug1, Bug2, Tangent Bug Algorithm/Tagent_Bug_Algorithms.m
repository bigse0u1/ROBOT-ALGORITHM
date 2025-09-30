% 구글링 多
clc; clear; close all;

start = [0, 0]; goal  = [10, 10];    % 시작점, 도착점

box = [ 2, 1, 2, 6;   % 첫 번째 장애물
        7, 6, 2, 5 ]; % 두 번째 장애물

p = start; step = 0.003; reached = false;   % reached: 도착

mode = "MOTION_TO_GOAL";   % 상태: MOTION_TO_GOAL 
M = [];                    % local minimum point
d_min = inf;               % goal까지의 최소 거리
last_dist = norm(p - goal);

numRays = 32;                           % 센서 설정
angles  = linspace(0,2*pi,numRays+1);   % 각도
angles(end) = [];
maxRange = 4.0;                         % 센서 최대 거리

rotRight = @(v,ang) (v*[cos(ang) -sin(ang); sin(ang) cos(ang)]);

figure; 
set(gcf, 'Color', 'k'); set(gca, 'Color', 'w'); set(gca, 'GridColor', 'k');
grid on; hold on; axis equal;
title('Tangent Bug Algorithms');
xlabel('X'); ylabel('Y');

plot(start(1), start(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');       % 시작점
plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');         % 도착점

for i=1:size(box,1)
    rectangle('Position',box(i,:),'FaceColor',[.8 .8 .8],'EdgeColor','k');
end

rh = plot(p(1),p(2),'bo','MarkerFaceColor','b','MarkerSize',7); % 로봇
ph = plot(p(1),p(2),'c-');                                      % 경로
mh = plot(NaN,NaN,'ks','MarkerFaceColor','k');                  % local minima

ray_free_h = plot(NaN,NaN,'g-');
ray_hit_h  = plot(NaN,NaN,'r-');

bSteps = 0;
while ~reached
    if norm(p - goal) < step, reached = true; break; 
    end

    ranges = scanEnv(p, angles, box, maxRange);
    is_free = (ranges == maxRange);

    xf=[]; yf=[]; xh=[]; yh=[];
    for k=1:numRays
        tip = p + ranges(k)*[cos(angles(k)) sin(angles(k))];
        if is_free(k)
            xf=[xf p(1) tip(1) NaN]; yf=[yf p(2) tip(2) NaN];
        else
            xh=[xh p(1) tip(1) NaN]; yh=[yh p(2) tip(2) NaN];
        end
    end
    set(ray_free_h,'XData',xf,'YData',yf);
    set(ray_hit_h, 'XData',xh,'YData',yh);

    if mode=="MOTION_TO_GOAL"
        bestDir=[]; bestDist=inf;
        for k=1:numRays
            if is_free(k)
                dir=[cos(angles(k)) sin(angles(k))];
                np = p + step*dir;
                dist=norm(np-goal);
                if dist<bestDist
                    bestDist=dist; bestDir=dir;
                end
            end
        end

        if isempty(bestDir)
            d = rotRight([1 0], -pi/12); % fallback 작은 회전
        else
            d = bestDir;
        end
        p = p + step*d;

        currDist = norm(p-goal);
        if any(~is_free) && currDist > last_dist+1e-5
            disp('Local minimum → BOUNDARY_FOLLOWING');
            mode="BOUNDARY_FOLLOWING";
            M=p; set(mh,'XData',M(1),'YData',M(2));
            d_min=currDist;
            bSteps=0;
            d=rotRight(d,-pi/12);
            continue;
        end
        last_dist=currDist;

    else
        bSteps=bSteps+1;
        d_min=min(d_min,norm(p-goal));

        d_leave=inf;
        for k=1:numRays
            if ~is_free(k)
                sensed=p+ranges(k)*[cos(angles(k)) sin(angles(k))];
                d_leave=min(d_leave,norm(sensed-goal));
            end
        end
        if d_leave < d_min
            mode="MOTION_TO_GOAL";
            last_dist=norm(p-goal);
            continue;
        end

        leftDir=rotRight(d,pi/12);
        if ~isInsideAny(p+step*leftDir,box)
            d=leftDir;
        else
            frontBlocked=isInsideAny(p+step*d,box);
            if frontBlocked, d=rotRight(d,-pi/12); 
            end
        end
        p=p+step*d;

        if ~isempty(M) && norm(p-M)<3*step && bSteps>50
            warning('Goal may be unreachable.'); 
            break;
        end
    end

    set(rh,'XData',p(1),'YData',p(2));
    set(ph,'XData',[get(ph,'XData') p(1)],'YData',[get(ph,'YData') p(2)]);
    drawnow limitrate;
end

if reached, disp('Goal Reached!'); 
end

function inside=isInsideAny(pt,boxes)
inside=any(pt(1)>=boxes(:,1)&pt(1)<=boxes(:,1)+boxes(:,3)& pt(2)>=boxes(:,2)&pt(2)<=boxes(:,2)+boxes(:,4));
end

function ranges=scanEnv(p,angles,boxes,maxR)
ranges=maxR*ones(size(angles));
    for k=1:length(angles)
        dir=[cos(angles(k)) sin(angles(k))];
        minDist=maxR;
        for m=1:size(boxes,1)
            t=rayRectIntersect(p,dir,boxes(m,:));
             if ~isempty(t)&&t<minDist, minDist=t; 
             end
        end
            ranges(k)=minDist;
    end
end

function tmin=rayRectIntersect(p,dir,rect)
x=rect(1);y=rect(2);w=rect(3);h=rect(4);
edges=[x,y,x+w,y; x+w,y,x+w,y+h; x+w,y+h,x,y+h; x,y+h,x,y];
tmin=[];
    for i=1:4
        t=raySegIntersect(p,dir,edges(i,1:2),edges(i,3:4));
        if ~isempty(t)&&(isempty(tmin)||t<tmin), tmin=t; 
        end
    end
end

function t=raySegIntersect(p,dir,a,b)
v1=p-a; v2=b-a; v3=[-dir(2),dir(1)];
den=dot(v2,v3);
    if abs(den)<1e-12, t=[]; 
        return; 
    end
t1=(v2(1)*v1(2)-v2(2)*v1(1))/den;
t2=dot(v1,v3)/den;
if t1>=0 && t2>=0 && t2<=1, t=t1; else, t=[]; end
end