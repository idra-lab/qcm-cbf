% Full quasi-conformal mapping
% Section 4.1.1: state mapping
% Section 4.2: input mapping (using the Jacobian)
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
% ALL_IN_BALL = true; % section 4.1.1 (state mapping)
ALL_IN_BALL = true; % section 4.2 (input mapping)
DT = 0.001;
T_MAX = 4e3;
POSITION_RADIUS = 'equal'; % 'move' or 'radius' or 'equal'

% build-up world
realWorld.domain.type = 'qc';
% Office's walls
wall1 = [15 0; 15 13; 13 13; 13 17; 15 17;15 19;13 19;13 20; 18 20; 18 19; 16 19; 16 0];
wall2 = [29 0; 29 19; 27 19; 27 20; 32 20; 32 19; 30 19; 30 0];
wall3 = [37 0; 37 19; 35 19; 35 20; 38 20; 38 0];
wall4 = [50 10; 40 10; 40 11; 50 11];
wall5 = [50 34; 40 34; 40 35; 50 35];
wall6 = [47 50; 47 47; 44 47; 44 50];
wall7 = [39 50; 39 40; 38 40; 38 43; 36 43; 36 44; 38 44; 38 50];
wall8 = [29 50; 29 43; 31 43; 31 42; 25 42; 25 43; 28 43; 28 50];
wall9 = [19 50; 19 46; 16 46; 16 50];
wall10 = [0 35; 10 35; 10 34; 0 34];
wall11 = [0 20; 7 20; 7 19; 0 19];
realWorld.domain.contour = [0 0; wall1; wall2; wall3; 47 0; 47 4; 50 4; wall4; wall5; 50 50; wall6; wall7; wall8; wall9; 0 50; wall10; wall11]';
realWorld.domain.contour = (1/10).*(realWorld.domain.contour - repmat([25;25], 1, size(realWorld.domain.contour, 2)));
%plot(realWorld.domain.contour(1,[1:end,1]), realWorld.domain.contour(2,[1:end,1]), 'LineWidth', 2)

realWorld.domain.goal = [0;-3];

ballWorld.domain.center = [0;0];
ballWorld.domain.radius = 5;
ballWorld.domain.goal = realWorld.domain.goal;

% real obstacles
realWorld.obstacles{1}.type = 'qc';
realWorld.obstacles{1}.contour =  [-1.8; -1.5] + [0.3*[-1 1 1 -1 -1]; 0.6*[1 1 -1 -1 1]];
ballWorld.obstacles{1}.center = [-1.8; -1.5];
ballWorld.obstacles{1}.radius = 0.5;

realWorld.obstacles{2}.type = 'qc';
realWorld.obstacles{2}.contour =  [2.05; -0.2] + [0.2*[-1 1 1 -1 -1]; 0.8*[1 1 -1 -1 1]];
ballWorld.obstacles{2}.center = [2.05; -0.25];
ballWorld.obstacles{2}.radius = 0.5;

realWorld.obstacles{3}.type = 'qc';
realWorld.obstacles{3}.contour =  [-0.4; 1.2] + [0.1*[-1 1 1 -1 -1]; 0.6*[1 1 -1 -1 1]];
ballWorld.obstacles{3}.center = [-0.4; 1.2];
ballWorld.obstacles{3}.radius = 0.5;

realWorld.obstacles{4}.type = 'qc';
realWorld.obstacles{4}.contour =  [0.85; -1.5] + 0.2*[-1 1 1 -1 -1; 1 1 -1 -1 1];
ballWorld.obstacles{4}.center = [0.85;-1.5];
ballWorld.obstacles{4}.radius = 0.5;

realWorld.obstacles{5}.type = 'qc';
realWorld.obstacles{5}.contour =  [-0.25; -1.5] + 0.2*[-1 1 1 -1 -1; 1 1 -1 -1 1];
ballWorld.obstacles{5}.center = [-0.25;-1.5];
ballWorld.obstacles{5}.radius = 0.5;

realWorld.obstacles{6}.type = 'qc';
realWorld.obstacles{6}.contour = [0.7; 1.2] + [-0.5 0.4 0.4 0.5 0.5 -0.5; 0 0 -1 -1 0.1 0.1];
ballWorld.obstacles{6}.center = [1.4;  1.9];
ballWorld.obstacles{6}.radius = 0.5;

realWorld.obstacles{7}.type = 'qc';
realWorld.obstacles{7}.contour =  [0.7; 0.7] + 0.2*[-1 1 1 -1 -1; 1 1 -1 -1 1];
ballWorld.obstacles{7}.center = [1.4; 0.7];
ballWorld.obstacles{7}.radius = 0.5;

realWorld.obstacles{8}.type = 'qc';
realWorld.obstacles{8}.contour =  [-1; 0.95] + [0.25*[-1 1 1 -1 -1]; 0.05*[1 1 -1 -1 1]];
ballWorld.obstacles{8}.center = [-1; .2];
ballWorld.obstacles{8}.radius = 0.5;

realWorld.obstacles{9}.type = 'qc';
realWorld.obstacles{9}.contour =  [-2; 2] + [0.25*[-1 1 1 -1 -1]; 0.05*[1 1 -1 -1 1]];
ballWorld.obstacles{9}.center = [-2; 2.1];
ballWorld.obstacles{9}.radius = 0.5;

realWorld.obstacles{10}.type = 'qc';
realWorld.obstacles{10}.contour =  [-2; 1.5] + [0.25*[-1 1 1 -1 -1]; 0.05*[1 1 -1 -1 1]];
ballWorld.obstacles{10}.center = [-2; 1];
ballWorld.obstacles{10}.radius = 0.5;

% % % real obstacles
realWorld.obstacles{1}.type = 'qc';
realWorld.obstacles{1}.contour =  [-1.8; -1.5] + [0.3*[-1 1 1 -1 -1]; 0.6*[1 1 -1 -1 1]];
ballWorld.obstacles{1}.center = [-1.8; -1.5];
ballWorld.obstacles{1}.radius = 0.5;

realWorld.obstacles{2}.type = 'qc';
realWorld.obstacles{2}.contour =  [2.05; -0.2] + [0.2*[-1 1 1 -1 -1]; 0.8*[1 1 -1 -1 1]];
ballWorld.obstacles{2}.center = 2*[2.05; -0.2];
ballWorld.obstacles{2}.radius = 0.5;

realWorld.obstacles{3}.type = 'qc';
realWorld.obstacles{3}.contour =  [-0.4; 1.2] + [0.1*[-1 1 1 -1 -1]; 0.6*[1 1 -1 -1 1]];
ballWorld.obstacles{3}.center = 2*[-0.4; 1.2];
ballWorld.obstacles{3}.radius = 0.5;

realWorld.obstacles{4}.type = 'qc';
realWorld.obstacles{4}.contour = [0.85; -1.5] + 0.2*[-1 1 1 -1 -1; 1 1 -1 -1 1];
ballWorld.obstacles{4}.center = 2*[0.85;-1.5];
ballWorld.obstacles{4}.radius = 0.5;

realWorld.obstacles{5}.type = 'qc';
realWorld.obstacles{5}.contour =  [-0.25; -1.5] + 0.2*[-1 1 1 -1 -1; 1 1 -1 -1 1];
ballWorld.obstacles{5}.center = 2*[-0.25;-1.5];
ballWorld.obstacles{5}.radius = 0.5;

realWorld.obstacles{6}.type = 'qc';
realWorld.obstacles{6}.contour = [0.7; 1.2] + [-0.5 0.4 0.4 0.5 0.5 -0.5; 0 0 -1 -1 0.1 0.1];
ballWorld.obstacles{6}.center = 2*[0.7; 1.2];
ballWorld.obstacles{6}.radius = 0.5;

realWorld.obstacles{7}.type = 'qc';
realWorld.obstacles{7}.contour = [0.7; 0.7] + 0.2*[-1 1 1 -1 -1; 1 1 -1 -1 1];
ballWorld.obstacles{7}.center = 2*[0.7; 0.7];
ballWorld.obstacles{7}.radius = 0.2;

realWorld.obstacles{8}.type = 'qc';
realWorld.obstacles{8}.contour = [-1; 0.95] + [0.25*[-1 1 1 -1 -1]; 0.05*[1 1 -1 -1 1]];
ballWorld.obstacles{8}.center = 2*[-1; 0.95];
ballWorld.obstacles{8}.radius = 0.5;

realWorld.obstacles{9}.type = 'qc';
realWorld.obstacles{9}.contour = [-2; 2] + [0.25*[-1 1 1 -1 -1]; 0.05*[1 1 -1 -1 1]];
ballWorld.obstacles{9}.center = 2*[-2; 2];
ballWorld.obstacles{9}.radius = 0.5;

realWorld.obstacles{10}.type = 'qc';
realWorld.obstacles{10}.contour = [-2; 1.5] + [0.25*[-1 1 1 -1 -1]; 0.05*[1 1 -1 -1 1]];
ballWorld.obstacles{10}.center = 2*[-2; 1.5];
ballWorld.obstacles{10}.radius = 0.5;


Nobst = numel(realWorld.obstacles);
for i = 1 : Nobst
    ballWorld.obstacles{i}.centerOriginal = ballWorld.obstacles{i}.center;
    ballWorld.obstacles{i}.radiusOriginal = ballWorld.obstacles{i}.radius;
end

% figure(2)
% hold on
% plot(realWorld.domain.contour(1,[1:end,1]), realWorld.domain.contour(2,[1:end,1]), 'LineWidth', 2)
% for i = 1 : Nobst
%     plot(realWorld.obstacles{i}.contour(1,[1:end,1]), realWorld.obstacles{i}.contour(2,[1:end,1]), 'LineWidth', 2)
% end


vertices_x = {realWorld.domain.contour(1,:)};
vertices_y = {realWorld.domain.contour(2,:)};
for i = 1 : Nobst
    vertices_x{end+1} = realWorld.obstacles{i}.contour(1,end:-1:1);
    vertices_y{end+1} = realWorld.obstacles{i}.contour(2,end:-1:1);
end

% real world
% x = [-2;2.25]; col_ = 1;
% x = [-2;2.25]; col_ = 2;
% x = [0.85;2.]; col_ = 3;
x = [1.8;2.]; col_ = 4;

vertices_all = {vertices_x, vertices_y};
wwo = WorldWithObstacleMappingQC(vertices_all, ballWorld.domain, ballWorld.obstacles);
wwo.storeState(x);
r2bMap = wwo.getReal2BallMapHandle();
b2rMap = wwo.getBall2RealMapHandle_new();

% robot, goal, and trajectory variables
xBall = r2bMap(x);

% xG = [2;-1.2];
% xG = [-1.9;-2.3];
% xG = [-1.9;-2.3];
xG = [0.9;-2];

xGBall = r2bMap(xG);
xTraj = zeros(2,T_MAX);
xTraj(:,1) = x;
xTrajBall = zeros(2,T_MAX);
xTrajBall(:,1) = xBall;

myColors{1} = [0.4660 0.6740 0.1880];
myColors{2} = [0.8500 0.3250 0.0980];
myColors{3} = [0.4940 0.1840 0.5560];
myColors{4} = [0.9290 0.6940 0.1250];

PLOT = true;

% plots
if PLOT
figure(1); %('units','normalized','position',[0 0 1 1])
%clf;
% subplot(1,2,1), 
hold on, axis equal, axis([-3 3 -3 3]), set(gca, 'Visible', 'off')
hSTraj = line(x(1), x(2), 'Color', myColors{col_}, 'LineWidth', 2);
hS = scatter(x(1), x(2), 'MarkerEdgeColor', [0 0.4470 0.7410], 'MarkerFaceColor', [0 0.4470 0.7410]);
plot(realWorld.domain.contour(1,[1:end,1]), realWorld.domain.contour(2,[1:end,1]), 'k', 'LineWidth', 2)
for i = 1 : Nobst
    plot(realWorld.obstacles{i}.contour(1,[1:end,1]), realWorld.obstacles{i}.contour(2,[1:end,1]), 'Color', 'k', 'LineWidth', 2)
end
scatter(xG(1), xG(2), 'MarkerEdgeColor', [0.6350 0.0780 0.1840], 'MarkerFaceColor', [0.6350 0.0780 0.1840]);

% subplot(1,2,2), hold on, axis equal, axis([-6 6 -6 6]), set(gca, 'Visible', 'off')
% hSBall = scatter(xBall(1), xBall(2), 'filled');
% hSTrajBall = line(xBall(1), xBall(2), 'LineWidth', 2);
% domainBall = ballWorld.domain.center + ballWorld.domain.radius * [cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];
% plot(domainBall(1,[1:end,1]), domainBall(2,[1:end,1]), 'LineWidth', 2)
% hObstBall = cell(1,numel(ballWorld.obstacles));
% for i = 1 : Nobst
%     obstBall = ballWorld.obstacles{i}.center + ballWorld.obstacles{i}.radius * [cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];
%     hObstBall{i} = plot(obstBall(1,[1:end,1]), obstBall(2,[1:end,1]), 'Color', [.5 0 .5], 'LineWidth', 2);
% end
% hGBall = scatter(xGBall(1), xGBall(2), 'filled');
F(1) = getframe(gcf) ;
drawnow
end

%pause

%% main loop
for t = 2 : T_MAX   
    %%% check obstacles
    
    %%% real to ball
    
    %xBall = r2bMap(x);
    
    if ALL_IN_BALL
        uNomBall = 2 * diag([6;1]) * (xGBall - xBall);
    else
        uNomBall = 10 * diag([1;1]) * (xGBall - xBall);
        %uNomReal = 10 * diag([6;1]) * (xG - x);
        %uNomBall = (jacobianest(r2bMap, x) + 1E-3*eye(2)) * uNomReal;
    end

    %%% controller synthesis
    % robot does not care about the obstacles
    uBall = uNomBall;
    % obstacles care about the robot
    uObstNomBall = zeros(2*Nobst,1);
    rhoObstNomBall = zeros(Nobst,1);
    for i = 1 : Nobst
        uObstNomBall(2*i-1:2*i) = 50*(ballWorld.obstacles{i}.centerOriginal - ballWorld.obstacles{i}.center);
        rhoObstNomBall(i) = 50*(ballWorld.obstacles{i}.radiusOriginal - ballWorld.obstacles{i}.radius);
    end
    Acbf = zeros(Nobst,3*Nobst);
    bcbf = zeros(Nobst,1);
    for i = 1 : Nobst
        Acbf(i,2*i-1:2*i) = 2*(xBall-ballWorld.obstacles{i}.center)';
        Acbf(i,2*Nobst+i) = 2*ballWorld.obstacles{i}.radius;
        bcbf(i) = 2*(xBall-ballWorld.obstacles{i}.center)'*uBall + 0.2e2 * (norm(xBall-ballWorld.obstacles{i}.center)^2-ballWorld.obstacles{i}.radius^2);
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
    %Jbefore = jacobianest(r2bMap, x);
    %wwo = WorldWithObstacleMappingQC(vertices_all, ballWorld.domain, ballWorld.obstacles);
    wwo.storeState(x);
    r2bMap = wwo.getReal2BallMapHandle();
    b2rMap = wwo.getBall2RealMapHandle_new();
    
    %norm(Jbefore - jacobianest(r2bMap, x))
    
    %%% integration step
    if ALL_IN_BALL
        xBall = xBall + uBall*DT;
        x = b2rMap(xBall);
        xBall = r2bMap(x);
    else
        % Both jacobian transpose and damped least square work here, but
        % damped least square cause small jump in the ball world. The 
        % transpose solution is different from ALL_IN_BALL 
        u = (jacobianest(r2bMap, x) + 1E-4*eye(2)) \  uBall; % Numerical computation with adaptive step
        % u = jacobianest(r2bMap, x)' * uBall;
        x = x + u*DT;
        xBall = r2bMap(x);
    end
    
    %%% plot
    if PLOT
        for i = 1 : numel(ballWorld.obstacles)
            obstBall = ballWorld.obstacles{i}.center + ballWorld.obstacles{i}.radius * [cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];
            hObstBall{i}.XData = obstBall(1,[1:end,1]);
            hObstBall{i}.YData = obstBall(2,[1:end,1]);
        end
    end
    
    xTraj(:,t) = x;
    xTrajBall(:,t) = xBall;
    
    % if t==1
    %     hS.XData = x(1);
    %     hS.YData = x(2);
    % end
    hSTraj.XData = xTraj(1,1:t);
    hSTraj.YData = xTraj(2,1:t);
    hSBall.XData = xBall(1);
    hSBall.YData = xBall(2);
    hSTrajBall.XData = xTrajBall(1,1:t);
    hSTrajBall.YData = xTrajBall(2,1:t);
    F(t) = getframe(gcf);
    drawnow
end