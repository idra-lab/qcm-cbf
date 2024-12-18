classdef WorldWithObstacleMappingQC < handle
    properties (Access=private)
        interior = false;
        vertices
        fx
        fy
        R
        xPrev
    end
    methods
        % constructor
        function this = WorldWithObstacleMappingQC(vertices, ballDomain, ballObstacles)
            this.vertices = polyshape(vertices{1}, vertices{2});
            tr = triangulation(this.vertices);
            model = createpde;
            geometryFromMesh(model, tr.Points', tr.ConnectivityList');
            generateMesh(model,'Hmax',0.1); % TODO Hmax is a tuning parameter (precision VS speed)
            v = model.Mesh.Nodes';
            f = model.Mesh.Elements';
            f = f(:,1:3);
            [f,v] = gpp_clean_mesh(f,v);
            bd = meshboundaries(f);
            c = [ballDomain.center'; cell2mat(cellfun(@(x) x.center, ballObstacles, 'UniformOutput', false))'];
            r = [ballDomain.radius; cell2mat(cellfun(@(x) x.radius, ballObstacles, 'UniformOutput', false))'];
            if numel(bd) > 2
                idx_ordered = zeros(numel(bd)-1,1);
                for i = 2 : numel(bd)
                    for j = 1 : numel(vertices{1})
                        if inpolygon(v(bd{i},1), v(bd{i},2), vertices{1}{j}, vertices{2}{j})
                            idx_ordered(i-1) = j;
                        end
                    end
                end
                c(2:end,:) = c(idx_ordered,:);
                r(2:end) = r(idx_ordered);
            end
            map = polygonal_to_ball_qc_map_prescribed_holes(v, f, bd, c, r);
            Nmax = size(map,1);
            if size(map,1) > size(v,1)
                Nmax = size(map,1)-1;
            end
            this.fx = TriScatteredInterp(v(:,1),v(:,2),map(1:Nmax,1));
            this.fy = TriScatteredInterp(v(:,1),v(:,2),map(1:Nmax,2));
        end
        function storeState(this, statePrev)
            this.xPrev = statePrev;
        end
        % getters
        function mapHandle = getReal2BallMapHandle(this)
            mapHandle = @(q) this.real2ball(q);
        end
        function mapHandle = getBall2RealMapHandle(this)
            mapHandle = @(xBall, xRealPrev) this.ball2real(xBall, xRealPrev);
        end
        function mapHandle = getBall2RealMapHandle_new(this)
            mapHandle = @(xBall) this.ball2real_new(xBall);
        end
    end
    methods (Access=private)
        function xBall = real2ball(this, xReal)
            xBall = [this.fx(xReal'); this.fy(xReal')];
        end
        function xReal = ball2real(this, xBall, xRealPrev)
            options = optimset('MaxFunEvals', 100, ...
                'MaxIter', 100, ...
                'TolFun', 1e-6, ...
                'TolX', 1e-6, ...
                'Display', 'off');
            directMapping = @(p) this.real2ball(p)-xBall;
            xReal = fsolve(directMapping, xRealPrev, options);
        end
        function xReal = ball2real_new(this, xBall)
            options = optimset('MaxFunEvals', 100, ...
                'MaxIter', 100, ...
                'TolFun', 1e-6, ...
                'TolX', 1e-6, ...
                'Display', 'off');
            directMapping = @(p) this.real2ball(p)-xBall;
            xReal = fsolve(directMapping, this.xPrev, options);
        end
    end
end