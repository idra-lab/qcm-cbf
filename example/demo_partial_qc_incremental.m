% Partial quasi-conformal mapping
% Section 4.1.2
clc
clear
close all

addpath(genpath('../'))

%% setup
% flags and constants
% ALL_IN_DISC = true : states are mapped from the real world to the
%                      disc world, and nominal and CBF-QP controllers are
%                      evaluated in the disc world (this way we can
%                      literally get the obstacles out of the way)
% ALL_IN_BALL = false : nominal controllers are evaluated in the real
%                       world and mapped to the ball world through the
%                       jacobian of the diffeomorphism, then CBF-QP
%                       controller is evaluated in the ball world and
%                       mapped back to the real world through the
%                       pseudo-inverse Jacobian
ALL_IN_BALL = true;
DT = 0.001;
T_MAX = 1e4;
POSITION_RADIUS = 'equal'; % 'move' or 'radius' or 'equal'
LAMBDA = 5e4; % LAMBDA should be big enough to ensure proper mapping

% build-up world
realWorld.domain.type = 'qc';
realWorld.domain.contour = 2.5*[-1 1 1 -1;
    -1 -1 1 1];
realWorld.domain.goal = [0;-5];
ballWorld.domain.center = [0;0];
ballWorld.domain.radius = 5;
ballWorld.domain.goal = realWorld.domain.goal;
real_obstacle_contours{1} = [-0.65; -0.75] + 0.25*[-1 1 1 -1 -1; 1 1 -1 -1 1];%[0.65; 0.15] + 0.5*[1 -1 0 1; 0 0 1 0];
real_obstacle_contours{2} = [0.65; 0.15] + 0.25*[1 -1 0 1; 0 0 1 0];%[-0.65; -0.75] + 0.5*[-1 1 1 -1 -1; 1 1 -1 -1 1];
ball_obstacle_centers{1} = [-0.8;-1.25];%[0.5;0.75];
ball_obstacle_centers{2} = [0.5;0.75];%[-0.8;-1.25];
ball_obstacle_radii{1} = 0.9;
ball_obstacle_radii{2} = 0.75;
obstacle_encountered = false(numel(real_obstacle_contours),1);
D_OBST_DETECTION = 1;
realWorld.obstacles = {};
ballWorld.obstacles = {};
Nobst = numel(realWorld.obstacles);

%% init
% real world
wm = WorldMapping(realWorld, ballWorld);
wm.evaluateMappings(LAMBDA)
[r2bMap, b2rMap, r2bJac, b2rJac] = wm.getMappings();

% robot, goal, and trajectory variables
x = [2;2];
xBall = r2bMap(x);
xG = [-0.25;-2];
xGBall = r2bMap(xG);
xTraj = zeros(2,T_MAX);
xTraj(:,1) = x;
xTrajBall = zeros(2,T_MAX);
xTrajBall(:,1) = xBall;

% plots
figure('units','normalized','position',[0 0 1 1])

subplot(1,2,1), hold on, axis equal, axis([-3 3 -3 3]), set(gca, 'Visible', 'off')
hS = scatter(x(1), x(2), 1000, '.');
hSTraj = line(x(1), x(2), 'LineWidth', 2);
plot(realWorld.domain.contour(1,[1:end,1]), realWorld.domain.contour(2,[1:end,1]), 'LineWidth', 2)
for i = 1 : Nobst
    plot(realWorld.obstacles{i}.contour(1,[1:end,1]), realWorld.obstacles{i}.contour(2,[1:end,1]), 'LineWidth', 2)
end
scatter(xG(1), xG(2), 1000, '.');

subplot(1,2,2), hold on, axis equal, axis([-6 6 -6 6]), set(gca, 'Visible', 'off')
hSBall = scatter(xBall(1), xBall(2), 1000, '.');
hSTrajBall = line(xBall(1), xBall(2), 'LineWidth', 2);
domainBall = ballWorld.domain.center + ballWorld.domain.radius * [cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];
plot(domainBall(1,[1:end,1]), domainBall(2,[1:end,1]), 'LineWidth', 2)
hObstBall = cell(1,numel(ballWorld.obstacles));
for i = 1 : Nobst
    obstBall = ballWorld.obstacles{i}.center + ballWorld.obstacles{i}.radius * [cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];
    hObstBall{i} = plot(obstBall(1,[1:end,1]), obstBall(2,[1:end,1]), 'LineWidth', 2);
end
hGBall = scatter(xGBall(1), xGBall(2), 1000, '.');

drawnow
% pause

%% main loop
for t = 1 : T_MAX
    %%% check obstacles
    for i = 1 : numel(real_obstacle_contours)
        if min(vecnorm(real_obstacle_contours{i}-x)) < D_OBST_DETECTION
            if ~obstacle_encountered(i)
                obstacle_encountered(i) = true;
                realWorld.obstacles{end+1}.type = 'qc';
                realWorld.obstacles{end}.contour = real_obstacle_contours{i};
                ballWorld.obstacles{end+1}.center = ball_obstacle_centers{i};
                ballWorld.obstacles{end}.centerOriginal = ballWorld.obstacles{end}.center;
                ballWorld.obstacles{end}.radius = ball_obstacle_radii{i};
                ballWorld.obstacles{end}.radiusOriginal = ballWorld.obstacles{end}.radius;
                
                subplot(1,2,1), hold on, axis equal, axis([-3 3 -3 3]), set(gca, 'Visible', 'off')
                plot(realWorld.obstacles{end}.contour(1,[1:end,1]), realWorld.obstacles{end}.contour(2,[1:end,1]), 'LineWidth', 2)
                subplot(1,2,2), hold on, axis equal, axis([-6 6 -6 6]), set(gca, 'Visible', 'off')
                obstBall = ballWorld.obstacles{end}.center + ballWorld.obstacles{end}.radius * [cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];
                hObstBall{end+1} = plot(obstBall(1,[1:end,1]), obstBall(2,[1:end,1]), 'LineWidth', 2);
                
                wm.setRealWorld(realWorld);
                wm.setBallWorld(ballWorld);
                wm.evaluateMappings(LAMBDA);
                [r2bMap, b2rMap, r2bJac, b2rJac] = wm.getMappings();
                
                xGBall = r2bMap(xG);
                hGBall.XData = xGBall(1);
                hGBall.YData = xGBall(2);
                
                Nobst = numel(realWorld.obstacles);
            end
        end
    end
    
    if (t==55)
        dd=1;
    end
    
    %%% real to ball
    xBall = r2bMap(x);
    if ALL_IN_BALL
        uNomBall = 10 * diag([6;1]) * (xGBall - xBall);
    else
        uNomReal = 10 * diag([6;1]) * (xG - x);
        Jr2b = AutoDiffJacobianFiniteDiff(r2bMap, x);%timeAD=toc;
        
        %uNomBall = r2bJac(x) * uNomReal;
        uNomBall = Jr2b * uNomReal;
    end
        
    %%% controller synthesis
    % robot does not care about the obstacles
    uBall = uNomBall;
    % obstacles care about the robot
    uObstNomBall = zeros(2*Nobst,1);
    rhoObstNomBall = zeros(Nobst,1);
    for i = 1 : Nobst
        uObstNomBall(2*i-1:2*i) = 10*(ballWorld.obstacles{i}.centerOriginal - ballWorld.obstacles{i}.center);
        rhoObstNomBall(i) = 10*(ballWorld.obstacles{i}.radiusOriginal - ballWorld.obstacles{i}.radius);
    end
    Acbf = zeros(Nobst,3*Nobst);
    bcbf = zeros(Nobst,1);
    for i = 1 : Nobst
        Acbf(i,2*i-1:2*i) = 2*(xBall-ballWorld.obstacles{i}.center)';
        Acbf(i,2*Nobst+i) = 2*ballWorld.obstacles{i}.radius;
        bcbf(i) = 2*(xBall-ballWorld.obstacles{i}.center)'*uBall + 0.5e2 * (norm(xBall-ballWorld.obstacles{i}.center)^2-ballWorld.obstacles{i}.radius^2);
    end
    if strcmp(POSITION_RADIUS, 'move')
        Wcenter = 1;
        Wradius = 1e3;
    elseif strcmp(POSITION_RADIUS, 'radius')
        Wcenter = 1e3;
        Wradius = 1;
    elseif strcmp(POSITION_RADIUS, 'equal')
        Wcenter = 1;
        Wradius = 1;
    end
    if Nobst == 0
        uObstBallrhoObstBall = [uObstNomBall; rhoObstNomBall];
    else
        uObstBallrhoObstBall = quadprog(2*blkdiag(Wcenter*eye(2*Nobst), Wradius*eye(Nobst)), ...
            -2*[Wcenter*uObstNomBall; Wradius*rhoObstNomBall]', ...
            Acbf, ...
            bcbf, ...
            [],[],[],[],[],optimoptions(@quadprog,'Display','off'));
    end
    for i = 1 : Nobst
        xDotI = uObstBallrhoObstBall(2*i-1:2*i);
        rDotI = uObstBallrhoObstBall(2*Nobst+i);
        ballWorld.obstacles{i}.center = ballWorld.obstacles{i}.center + xDotI*DT;
        ballWorld.obstacles{i}.radius = ballWorld.obstacles{i}.radius + rDotI*DT;
    end
    
    %%% re-evaluate mappings after changing obstacles
    wm.setBallWorld(ballWorld);
    wm.composeMappings(LAMBDA);
    [r2bMap, b2rMap, r2bJac, b2rJac] = wm.getMappings();
    
    %%% integration step
    if ALL_IN_BALL
        xBall = xBall + uBall*DT;
        x = b2rMap(xBall, x);
    else
        Jr2b = AutoDiffJacobianFiniteDiff(r2bMap, x);%timeAD=toc;
        
%         u = r2bJac(x) \ uBall;
        u = Jr2b \ uBall;
        x = x + u*DT;
    end
    
    %%% plot
    for i = 1 : numel(ballWorld.obstacles)
        obstBall = ballWorld.obstacles{i}.center + ballWorld.obstacles{i}.radius * [cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];
        hObstBall{i}.XData = obstBall(1,[1:end,1]);
        hObstBall{i}.YData = obstBall(2,[1:end,1]);
    end
    
    xTraj(:,t) = x;
    xTrajBall(:,t) = xBall;
    
    hS.XData = x(1);
    hS.YData = x(2);
    hSTraj.XData = xTraj(1,1:t);
    hSTraj.YData = xTraj(2,1:t);
    hSBall.XData = xBall(1);
    hSBall.YData = xBall(2);
    hSTrajBall.XData = xTrajBall(1,1:t);
    hSTrajBall.YData = xTrajBall(2,1:t);
    drawnow
end
