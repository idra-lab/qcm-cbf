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
LAMBDA = 1e0;

% build-up world
realWorld.domain.type = 'qc';
realWorld.domain.contour = 2.5*[-1 1 1 -1;
    -1 -1 1 1];
realWorld.domain.goal = [0;-2];
realWorld.obstacles{1}.type = 'qc';
realWorld.obstacles{1}.contour = [-0.5; 0.25] + 1.1*[-0.5 -0.4 -0.4 0.4 0.4 0.5 0.5 -0.5;
    -0.5 -0.5 0 0 -0.5 -0.5 0.1 0.1];
ballWorld.domain.center = [0;0];
ballWorld.domain.radius = 5;
ballWorld.domain.goal = realWorld.domain.goal;
ballWorld.obstacles{1}.center = [-0.95;0.25];
ballWorld.obstacles{1}.centerOriginal = ballWorld.obstacles{1}.center;
ballWorld.obstacles{1}.radius = 0.75;
ballWorld.obstacles{1}.radiusOriginal = ballWorld.obstacles{1}.radius;

%% init
% real world
wm = WorldMapping(realWorld, ballWorld);
wm.evaluateMappings(LAMBDA)
[r2bMap, b2rMap, r2bJac, b2rJac] = wm.getMappings();

% robot, goal, and trajectory variables
x = [2;2];
xBall = r2bMap(x);
xG = [-0.5;-1];
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
for i = 1 : numel(realWorld.obstacles)
    plot(realWorld.obstacles{i}.contour(1,[1:end,1]), realWorld.obstacles{i}.contour(2,[1:end,1]), 'LineWidth', 2)
end
scatter(xG(1), xG(2), 1000, '.');

subplot(1,2,2), hold on, axis equal, axis([-6 6 -6 6]), set(gca, 'Visible', 'off')
hSBall = scatter(xBall(1), xBall(2), 1000, '.');
hSTrajBall = line(xBall(1), xBall(2), 'LineWidth', 2);
domainBall = ballWorld.domain.center + ballWorld.domain.radius * [cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];
plot(domainBall(1,[1:end,1]), domainBall(2,[1:end,1]), 'LineWidth', 2)
hObstBall = cell(1,numel(ballWorld.obstacles));
for i = 1 : numel(ballWorld.obstacles)
    obstBall = ballWorld.obstacles{i}.center + ballWorld.obstacles{i}.radius * [cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];
    hObstBall{i} = plot(obstBall(1,[1:end,1]), obstBall(2,[1:end,1]), 'LineWidth', 2);
end
scatter(xGBall(1), xGBall(2), 1000, '.');

drawnow

%% main loop
for t = 1 : T_MAX
    %%% real to ball
    xBall = r2bMap(x);
    if ALL_IN_BALL
        uNomBall = 5 * diag([6;1]) * (xGBall - xBall);
    else
        uNomReal = 5 * diag([6;1]) * (xG - x);
        uNomBall = r2bJac(x) * uNomReal;
    end
    
    %%% controller synthesis
    % robot does not care about the obstacles
    uBall = uNomBall;
    % obstacles care about the robot (hardcoded for 1 obstacle)
    uObstNomBall = 10*(ballWorld.obstacles{1}.centerOriginal - ballWorld.obstacles{1}.center);
    rhoObstNomBall = 10*(ballWorld.obstacles{1}.radiusOriginal - ballWorld.obstacles{1}.radius);
    % Acbf = [2*(xBall-ballWorld.obstacles{1}.center)', 2*ballWorld.obstacles{1}.radius];
    % bcbf = 2*(xBall-ballWorld.obstacles{1}.center)'*uBall + 1e2 * (norm(xBall-ballWorld.obstacles{1}.center)^2-ballWorld.obstacles{1}.radius^2);
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
    Aqp = 2*blkdiag(Wcenter*eye(2), Wradius);
    Bqp = -2*[Wcenter*uObstNomBall', Wradius*rhoObstNomBall]';
    safetyMargin = 1.5;
    Cqp = [-2*(xBall-ballWorld.obstacles{1}.center)', -2*(safetyMargin*ballWorld.obstacles{1}.radius)]';
    Dqp = 2*(xBall-ballWorld.obstacles{1}.center)'*uBall + 1e2 * (norm(xBall-ballWorld.obstacles{1}.center)^2-(safetyMargin*ballWorld.obstacles{1}.radius)^2);
    if Dqp >=0
        uRhoObstBall = -Aqp\Bqp;
    else
        uRhoObstBall = Aqp\(-Bqp+(Cqp'*(Aqp\Bqp)-Dqp)/(Cqp'*(Aqp\Cqp))*Cqp);
    end
    uObstBall = uRhoObstBall(1:2);
    rhoObstBall = uRhoObstBall(3);
    ballWorld.obstacles{1}.center = ballWorld.obstacles{1}.center + uObstBall*DT;
    ballWorld.obstacles{1}.radius = ballWorld.obstacles{1}.radius + rhoObstBall*DT;
    
    %%% re-evaluate mappings after changing obstacles
    wm.setBallWorld(ballWorld);
    wm.composeMappings(LAMBDA);
    [r2bMap, b2rMap, r2bJac, b2rJac] = wm.getMappings();
    
    %%% integration step
    if ALL_IN_BALL
        xBall = xBall + uBall*DT;
        x = b2rMap(xBall, x);
    else
        u = r2bJac(x) \ uBall;
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
