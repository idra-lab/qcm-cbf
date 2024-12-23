clear;
close all;
clc;

addpath(genpath('../'))

%% Pan-Tilt camera
l0 = 0;
l1 = 1; % Link 1 length
l2 = 1; % Link 2 length
lAll = [l0; l1; l2];

ptLimits.qmin = [-pi, -pi]/5;
ptLimits.qmax = [pi, pi]/5;

myColors{1} = [0.8500 0.3250 0.0980];
myColors{2} = [0.9290 0.6940 0.1250];
myColors{3} = [0.4940 0.1840 0.5560];
myColors{4} = [0.4660 0.6740 0.1880];
myColors{5} = [0 0.4470 0.7410]; % Default blue

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
T_MAX = 2e3;
POSITION_RADIUS = 'equal'; % 'move' or 'radius' or 'equal'

% build-up worlds
cartWorld.domain.type = 'qc';

% cart_obstacle_center{1} = [1.1; -0.05];
% alpha = deg2rad(0);

cart_obstacle_center{1} = [0.7; 1];
alpha = deg2rad(-50);

cartWorld.obstacles{1}.contour = cart_obstacle_center{1} + 0.3*[-0.5 -0.3 -0.3 0.3 0.3 0.5 0.5 -0.5;
    -0.5 -0.5 0 0 -0.5 -0.5 0.2 0.2];

Ralpha = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)];
cartWorld.obstacles{1}.contour = Ralpha*cartWorld.obstacles{1}.contour;

cart_obstacle_center{2} = [0.7;-0.15];
cartWorld.obstacles{2}.contour = cart_obstacle_center{2}+ 0.15*[1 -1 0 1; 0 0 1 0]; %0.2*[cos((0:3)*2*pi/4);sin((0:3)*2*pi/4)];

ptWorld.domain.type = 'qc';
ptWorld.domain.contour = ptLimits.qmax(1)*[-1 1 1 -1; -1 -1 1 1];
ptWorld.domain.goal = [0.5; 0.5];

% Cart world contour
cartWorld.domain.contour = [];
for j=1:length(ptWorld.domain.contour)
    Tcont = fk_PT(ptWorld.domain.contour(:,j), lAll); 
    cartWorld.domain.contour = [cartWorld.domain.contour, Tcont(1:2,4)]; 
end

% Get a reachable Cartesian goal
Tgoal = fk_PT(ptWorld.domain.goal, lAll); 
cartWorld.domain.goal = Tgoal(1:2,4);

for i = 1:numel(cartWorld.obstacles)
    ptWorld.obstacles{i}.contour = [];
    for j=1:length(cartWorld.obstacles{i}.contour)
        cartPt = cartWorld.obstacles{i}.contour(:,j);
        pt_ = ik_PT([cartPt; -0.7], lAll);
        ptWorld.obstacles{i}.contour = [ptWorld.obstacles{i}.contour, pt_(1:2)];
    end
    
    pt_obstacle_center{i} = mean(ptWorld.obstacles{i}.contour, 2);
end

% Forward kinematics seems inaccurate

cart_obstacle_center{1} = [0.65; 1.1];
cartWorld.obstacles{1}.contour = cart_obstacle_center{1} + 0.25*[-0.5 -0.3 -0.3 0.3 0.3 0.5 0.5 -0.5;
    -0.5 -0.5 0 0 -0.5 -0.5 0.2 0.2];

Ralpha = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)];
cartWorld.obstacles{1}.contour = Ralpha*cartWorld.obstacles{1}.contour;

cart_obstacle_center{2} = [0.7;-0.15];
cartWorld.obstacles{2}.contour = cart_obstacle_center{2}+ 0.1*[1 -1 0 1; 0 0 1 0]; %0.2*[cos((0:3)*2*pi/4);sin((0:3)*2*pi/4)];

ballWorld.domain.center = [0;0];
ballWorld.domain.radius = 2*ptLimits.qmax(1);
ballWorld.domain.goal = ptWorld.domain.goal;
radii = [0.45; 0.3];
for i=1:numel(ptWorld.obstacles)
    ballWorld.obstacles{i}.center = 2*pt_obstacle_center{i};
    ballWorld.obstacles{i}.centerOriginal = ballWorld.obstacles{i}.center;
    ballWorld.obstacles{i}.radius = radii(i);
    ballWorld.obstacles{i}.radiusOriginal = ballWorld.obstacles{i}.radius;
end

%% init
% real world
vertices_x = {ptWorld.domain.contour(1,:)};
vertices_y = {ptWorld.domain.contour(2,:)};
%%% Add obstacle
for i=1:numel(ptWorld.obstacles)
    vertices_x{end+1} = ptWorld.obstacles{i}.contour(1,end:-1:1);
    vertices_y{end+1} = ptWorld.obstacles{i}.contour(2,end:-1:1);
end
vertices_all = {vertices_x, vertices_y};
%% Create map handles
wwo = WorldWithObstacleMappingQC(vertices_all, ballWorld.domain, ballWorld.obstacles);
r2bMap = wwo.getReal2BallMapHandle();
b2rMap = wwo.getBall2RealMapHandle();

% robot, goal, and trajectory variables
x = [-0.4; -0.5];
% Get a Cartesian point
T0 = fk_PT(x, lAll); 
xCart = T0(1:2,4);

xBall = r2bMap(x);
xGBall = r2bMap(ptWorld.domain.goal);
xTrajCart = zeros(2,T_MAX);
T0 = fk_PT(x, lAll); 
xTrajCart(:,1) = T0(1:2,4);

xTrajPT = zeros(2,T_MAX);
xTrajPT(:,1) = x;
xTrajBall = zeros(2,T_MAX);
xTrajBall(:,1) = xBall;

PLOT = 1;

% plots
if PLOT
figure(1) %,'units','normalized'); %,'position',[0 0 1 1])
clf;
subplot(1,3,1), hold on, axis equal, axis([0.4 1.6 -0.5 0.5]), set(gca, 'Visible', 'off')
hSCart = scatter(xCart(1), xCart(2), 1000, '.', 'Color', myColors{5});
hSCartTraj = line(xCart(1), xCart(2), 'LineWidth', 2, 'Color', myColors{5});
plot(cartWorld.domain.contour(1,[1:end,1]), cartWorld.domain.contour(2,[1:end,1]), 'LineWidth', 2, 'Color', myColors{1})
for i = 1 : numel(cartWorld.obstacles)
    plot(cartWorld.obstacles{i}.contour(1,[1:end,1]), cartWorld.obstacles{i}.contour(2,[1:end,1]), 'LineWidth', 2, 'Color', myColors{i+1})
end
scatter(cartWorld.domain.goal(1), cartWorld.domain.goal(2), 1000, '.');

subplot(1,3,2), hold on, axis equal, axis(1.1*[ptLimits.qmin(1) ptLimits.qmax(1) ptLimits.qmin(2) ptLimits.qmax(2)]), set(gca, 'Visible', 'off')
hSPT = scatter(x(1), x(2), 1000, '.', 'Color', myColors{5});
hSPTTraj = line(x(1), x(2), 'LineWidth', 2, 'Color', myColors{5});
plot(ptWorld.domain.contour(1,[1:end,1]), ptWorld.domain.contour(2,[1:end,1]), 'LineWidth', 2, 'Color', myColors{1})
for i = 1 : numel(ptWorld.obstacles)
    plot(ptWorld.obstacles{i}.contour(1,[1:end,1]), ptWorld.obstacles{i}.contour(2,[1:end,1]), 'LineWidth', 2, 'Color', myColors{i+1})
end
scatter(ptWorld.domain.goal(1), ptWorld.domain.goal(2), 1000, '.');

subplot(1,3,3), hold on, axis equal, axis(2.2*[ptLimits.qmin(1) ptLimits.qmax(1) ptLimits.qmin(2) ptLimits.qmax(2)]), set(gca, 'Visible', 'off')
hSBall = scatter(xBall(1), xBall(2), 1000, '.', 'Color', myColors{5});
hSTrajBall = line(xBall(1), xBall(2), 'LineWidth', 2, 'Color', myColors{5});
domainBall = ballWorld.domain.center + ballWorld.domain.radius * [cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];
plot(domainBall(1,[1:end,1]), domainBall(2,[1:end,1]), 'LineWidth', 2, 'Color', myColors{1})
hObstBall = cell(1,numel(ballWorld.obstacles));

for i = 1 : numel(ballWorld.obstacles)
    obstBall = ballWorld.obstacles{i}.center + ballWorld.obstacles{i}.radius * [cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];
    hObstBall{i} = plot(obstBall(1,[1:end,1]), obstBall(2,[1:end,1]), 'LineWidth', 2, 'Color', myColors{i+1});
end

hGBall = scatter(xGBall(1), xGBall(2), 1000, '.');

F(1) = getframe(gcf);
drawnow
end

Nobst = numel(ballWorld.obstacles);
xG = ptWorld.domain.goal;
%% main loop
for t = 1 : T_MAX   
    %%% real to ball
    xBall = r2bMap(x);
    if ALL_IN_BALL
        uNomBall = 5 * diag([1;1]) * (xGBall - xBall);
    else
        uNomReal = 2 * diag([1;1]) * (xG - x);
        uNomBall = jacobianest(r2bMap, x) * uNomReal;
        
        %uNomBall = 2 * diag([1;1]) * (xGBall - xBall);
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
        bcbf(i) = 2*(xBall-ballWorld.obstacles{i}.center)'*uBall + 0.1e2 * (norm(xBall-ballWorld.obstacles{i}.center)^2-ballWorld.obstacles{i}.radius^2);
    end
    
    if strcmp(POSITION_RADIUS, 'move')
        Wcenter = 1;
        Wradius = 1e3;
    elseif strcmp(POSITION_RADIUS, 'radius')
        Wcenter = 1e2;
        Wradius = 1;
    elseif strcmp(POSITION_RADIUS, 'equal')
        Wcenter = 1;
        Wradius = 1;
    end

    uObstBallrhoObstBall = quadprog(2*blkdiag(Wcenter*eye(2*Nobst), Wradius*eye(Nobst)), ...
        -2*[Wcenter*uObstNomBall; Wradius*rhoObstNomBall]', ...
        Acbf, ...
        bcbf, ...
        [],[],[],[],[],optimoptions(@quadprog,'Display','off'));

    for i = 1 : Nobst
        xDotI = uObstBallrhoObstBall(2*i-1:2*i);
        rDotI = uObstBallrhoObstBall(2*Nobst+i);
        ballWorld.obstacles{i}.center = ballWorld.obstacles{i}.center + xDotI*DT;
        ballWorld.obstacles{i}.radius = ballWorld.obstacles{i}.radius + rDotI*DT;
    end
    
    %%% re-evaluate mappings after changing obstacles
    %Jbefore = jacobianest(r2bMap, x);
    
    wwo = WorldWithObstacleMappingQC(vertices_all, ballWorld.domain, ballWorld.obstacles);
    r2bMap = wwo.getReal2BallMapHandle();
    b2rMap = wwo.getBall2RealMapHandle();
    
%     Jdiff(t,:) = reshape(Jbefore \ jacobianest(r2bMap, x), 1, 4);
    
    %%% integration step
    if ALL_IN_BALL
        xBall = xBall + uBall*DT;
        x = b2rMap(xBall, x);
    else
        % u = AutoDiffJacobianFiniteDiff(r2bMap, x)' * uBall; 
        %xBall = xBall + uBall*DT;
        u = jacobianest(r2bMap, x) \ uBall; % Numerical computation with adaptive step
        x = x + u*DT;
    end
        
    
    %%% plot
    if PLOT
        for i = 1 : Nobst
            obstBall = ballWorld.obstacles{i}.center + ballWorld.obstacles{i}.radius * [cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];
            hObstBall{i}.XData = obstBall(1,[1:end,1]);
            hObstBall{i}.YData = obstBall(2,[1:end,1]);
        end
    end
    
    T0 = fk_PT(x, lAll); 
    xTrajCart(:,t) = T0(1:2,4);
    xTrajPT(:,t) = x;
    xTrajBall(:,t) = xBall;

    hSCart.XData = T0(1,4);
    hSCart.YData = T0(2,4);
    hSCartTraj.XData = xTrajCart(1,1:t);
    hSCartTraj.YData = xTrajCart(2,1:t);
    
    hSPT.XData = x(1);
    hSPT.YData = x(2);
    hSPTTraj.XData = xTrajPT(1,1:t);
    hSPTTraj.YData = xTrajPT(2,1:t);
    hSBall.XData = xBall(1);
    hSBall.YData = xBall(2);
    hSTrajBall.XData = xTrajBall(1,1:t);
    hSTrajBall.YData = xTrajBall(2,1:t);


    F(t) = getframe(gcf) ;
    drawnow
end