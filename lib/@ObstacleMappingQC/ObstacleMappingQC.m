classdef ObstacleMappingQC < handle
    properties (Access=private)
        interior = false;
        vertices
        fx
        fy
        R
    end
    methods
        % constructor
        function this = ObstacleMappingQC(vertices, inOut, th)
            if strcmp(inOut,'interior')
                this.interior = true;
                this.vertices = polyshape({vertices(1,:)}, {vertices(2,:)});
            elseif strcmp(inOut,'exterior')
                this.vertices = polyshape({min(vertices(1,:))+10*(max(vertices(1,:))-min(vertices(1,:)))*[-1 1 1 -1], vertices(1,:)}, ...
                    {min(vertices(2,:))+10*(max(vertices(2,:))-min(vertices(2,:)))*[-1 -1 1 1], vertices(2,:)});
            end
            this.evalRotMat(th);
            tr = triangulation(this.vertices);
            model = createpde;
            geometryFromMesh(model, tr.Points', tr.ConnectivityList');
            generateMesh(model,'Hmax',0.1);
            v = model.Mesh.Nodes';
            f = model.Mesh.Elements';
            f = f(:,1:3);
            [f,v] = gpp_clean_mesh(f,v);
            if this.interior
                map = polygonal_to_ball_qc_map_2d(v,f);
            else
                bd = meshboundaries(f);
                id = bd{end};
                p = v(id,:);
                map = polygon_to_disk_conformal_map_v2(p,v);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % colors = ['r','g','b','m','y','c','r','g','b','m','y','c','r','g','b','m','y','c','r','g','b','m','y','c','r','g','b','m','y','c','r','g','b','m','y','c']';
                % figure
                % subplot(1,2,1)
                % hold on
                % axis equal tight off
                % patch('Faces',f,'Vertices',v,'FaceColor','None','LineWidth',1)
                % % plot(p(:,1),p(:,2),'ro')
                % for i = 1 : min(size(colors,1),size(p,1))
                %     plot(p(i,1),p(i,2),'.','Color',colors(i),'MarkerSize',50)
                % end
                % subplot(1,2,2)
                % hold on
                % axis equal tight off
                % patch('Faces',f,'Vertices',map,'FaceColor','None','LineWidth',1)
                % % plot(map(id,1),map(id,2),'ro')
                % for i = 1 : min(size(colors,1),size(p,1))
                %     pi = this.R*map(id(i),:)';
                %     plot(pi(1),pi(2),'.','Color',colors(i),'MarkerSize',50)
                % end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
            Nmax = size(map,1);
            if size(map,1) > size(v,1)
                Nmax = size(map,1)-1;
            end
            this.fx = TriScatteredInterp(v(:,1),v(:,2),map(1:Nmax,1));
            this.fy = TriScatteredInterp(v(:,1),v(:,2),map(1:Nmax,2));
        end
        % getters
        function mapHandle = getReal2BallMapHandle(this)
            mapHandle = @(q) this.real2ball(q);
        end
    end
    methods (Access=private)
        function xBall = real2ball(this, xReal)
            xBall = this.R*[this.fx(xReal'); this.fy(xReal')];
        end
        function evalRotMat(this, th)
            this.R = [cos(th) -sin(th); sin(th) cos(th)];
        end
    end
end