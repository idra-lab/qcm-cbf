classdef WorldMapping < handle
    properties (Access=private)
        realWorld
        ballWorld
        real2BallMapping
        ball2RealMapping
        real2BallJacobian
        ball2RealJacobian
        epsilon = 1e-6;
        lambda
        f
        beta
    end
    methods
        % constructor
        function this = WorldMapping(realWorld, ballWorld)
            % realWorld.domain.contour = 2xNd matrix of vertices
            % realWorld.domain.center = 2x1 vector
            % realWorld.domain.goal = 2x1 vector
            % realWorld.obstacles{i} = 2xNi matrix of vertices
            % ballWorld.domain.center = 2x1 vector
            % ballWorld.domain.radius = positive real number
            % ballWorld.domain.goal = 2x1 vector
            % ballWorld.obsacles{i}.center = 2x1 vector
            % ballWorld.obsacles{i}.radius = positive real number
            this.realWorld = realWorld;
            this.ballWorld = ballWorld;
        end
        % getters
        function [p2bMap, b2pMap, p2bJac, b2pJac] = getMappings(this)
            p2bMap = this.real2BallMapping;
            b2pMap = this.ball2RealMapping;
            p2bJac = this.real2BallJacobian;
            b2pJac = this.ball2RealJacobian;
        end
        % setters
        function setRealWorld(this, realWorld)
            this.realWorld = realWorld;
        end
        function setBallWorld(this, ballWorld)
            this.ballWorld = ballWorld;
        end
        % mappings
        function evaluateObstacleMappings(this)
            M = numel(this.realWorld.obstacles);
            this.f = cell(1,M+1);
            this.beta = cell(1,M+1);
            for i = 0 : M
                if i == 0
                    % environment
                    % this.f{i+1} = ObstacleMappingExample(this.realWorld.domain.contour, 'interior').getReal2BallMapHandle();
                    if strcmp(this.realWorld.domain.type, 'sls') % sublevel set
                        this.f{i+1} = ObstacleMappingSLS(this.realWorld.domain, '').getReal2BallMapHandle();
                    elseif strcmp(this.realWorld.domain.type,'qc') % quasiconformal mapping
                        if isfield(this.realWorld.domain, 'angle')
                            this.f{i+1} = ObstacleMappingQC(this.realWorld.domain.contour, 'interior', this.realWorld.domain.angle).getReal2BallMapHandle();
                        else
                            this.f{i+1} = ObstacleMappingQC(this.realWorld.domain.contour, 'interior', 0).getReal2BallMapHandle();
                        end
                    elseif strcmp(this.realWorld.domain.type, 'sc') % Schwarz-Christoffel
                        this.f{i+1} = ObstacleMappingSC(this.realWorld.domain.contour, 'interior').getReal2BallMapHandle();
                    end
                    this.beta{i+1} = @(q) 1 - norm(this.f{i+1}(q))^2;
                else
                    % obstacles
                    % this.f{i+1} = ObstacleMappingExample(this.realWorld.obstacles{i}, 'exterior').getReal2BallMapHandle();
                    if strcmp(this.realWorld.domain.type, 'sls') % sublevel set
                        this.f{i+1} = ObstacleMappingSLS(this.realWorld.obstacles{i}, '').getReal2BallMapHandle();
                    elseif strcmp(this.realWorld.domain.type,'qc') % quasiconformal mapping
                        if isfield(this.realWorld.obstacles{i}, 'angle')
                            this.f{i+1} = ObstacleMappingQC(this.realWorld.obstacles{i}.contour, 'exterior', this.realWorld.obstacles{i}.angle).getReal2BallMapHandle();
                        else
                            this.f{i+1} = ObstacleMappingQC(this.realWorld.obstacles{i}.contour, 'exterior', 0).getReal2BallMapHandle();
                        end
                    elseif strcmp(this.realWorld.domain.type, 'sc') % Schwarz-Christoffel
                        this.f{i+1} = ObstacleMappingSC(this.realWorld.obstacles{i}.contour, 'exterior').getReal2BallMapHandle();
                    end
                    this.beta{i+1} = @(q) norm(this.f{i+1}(q))^2 - 1;
                end
            end
        end
        function composeMappings(this, lambda)
            this.lambda = lambda;
            this.evaluateReal2BallMapping();
            this.evaluateBall2RealMapping();
            this.evaluateReal2BallJacobian();
            this.evaluateBall2RealJacobian();
        end
        function evaluateMappings(this, lambda)
            this.evaluateObstacleMappings();
            this.composeMappings(lambda);
        end
    end
    methods (Access=private)
        function evaluateReal2BallMapping(this)
            M = numel(this.realWorld.obstacles);
            sigma = cell(1,M+1);
            for i = 0 : M
                sigma{i+1} = this.getSigmaFunctionHandle(i);
            end
            this.real2BallMapping = @hFunction;
            function pValue = hFunction(qValue)
                sigmaD = 1;
                for j = 0 : M
                    sigmaD = sigmaD - sigma{j+1}(qValue);
                end
                pValue = sigmaD*((qValue-this.realWorld.domain.goal)+this.ballWorld.domain.goal);
                for j = 0 : M
                    if j == 0
                        pValue = pValue + sigma{j+1}(qValue)*(this.ballWorld.domain.radius*this.f{j+1}(qValue)+this.ballWorld.domain.center);
                    else
                        pValue = pValue + sigma{j+1}(qValue)*(this.ballWorld.obstacles{j}.radius*this.f{j+1}(qValue)+this.ballWorld.obstacles{j}.center);
                    end
                end
            end
        end
        function evaluateBall2RealMapping(this)
            options = optimset('MaxFunEvals', 100, ...
                'MaxIter', 100, ...
                'TolFun', 1e-6, ...
                'TolX', 1e-6, ...
                'Display', 'off');
            this.ball2RealMapping = @inverseMapping;
            function qValue = inverseMapping(pValue, qValuePrev)
                directMapping = @(p) this.real2BallMapping(p)-pValue;
                qValue = fsolve(directMapping, qValuePrev, options);
            end
        end
        function evaluateReal2BallJacobian(this)
            this.real2BallJacobian = @(q) [this.real2BallMapping(q+[this.epsilon;0])-this.real2BallMapping(q-[this.epsilon;0]), this.real2BallMapping(q+[0;this.epsilon])-this.real2BallMapping(q-[0;this.epsilon])] / 2 / this.epsilon;
        end
        function evaluateBall2RealJacobian(this)
            this.ball2RealJacobian = @(p,q) [this.ball2RealMapping(p+[this.epsilon;0],q)-this.ball2RealMapping(p-[this.epsilon;0],q), this.ball2RealMapping(p+[0;this.epsilon],q)-this.ball2RealMapping(p-[0;this.epsilon],q)] / 2 / this.epsilon;
        end
        function sigmaFunctionHandle = getSigmaFunctionHandle(this, j)
            betaJBar = this.getBetaJBarFunctionHandle(j);
            sigmaFunctionHandle = @sigmaFunction;
            function sigmaValue = sigmaFunction(qValue)
                sigmaValue = norm(qValue-this.realWorld.domain.goal)^2 * betaJBar(qValue) / ...
                    (norm(qValue-this.realWorld.domain.goal)^2 * betaJBar(qValue) + this.lambda * this.beta{j+1}(qValue));
            end
        end
        function betaJBarFunctionHandle = getBetaJBarFunctionHandle(this, j)
            M = numel(this.beta)-1;
            betaJBarFunctionHandle = @betaJBarFunction;
            function betaJBarValue = betaJBarFunction(qValue)
                betaJBarValue = 1;
                for i = 0 : M
                    if i ~= j
                        betaJBarValue = betaJBarValue * this.beta{i+1}(qValue);
                    end
                end
            end
        end
    end
end