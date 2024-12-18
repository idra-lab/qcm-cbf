function map = polygonal_to_ball_qc_map_prescribed_holes(v,f,bd,c,r)
% Compute a quasi-conformal map of a multiply-connected open mesh onto 
% the unit disk with circular holes, where the circle centers and radii are
% prescribed
%
% Input:
% v: nv x 2 vertex coordinates of a simply-connected open triangle mesh
% f: nf x 3 triangulations of a simply-connected open triangle mesh
% bd: nb x 1 cell array of boundary vertex indices
% c: nb x 2 target coordinates of the circle centers (including the outermost boundary)
% r: nb x 1 target radii of the circles (including the outermost boundary)
%
% Output:
% map: nv x 2 vertex coordinates of the disk with circular holes
%
% Remark:
% Use bd = meshboundaries(f) to determine the boundaries first and follow
% their order to set up c and r
%
% Written by Gary P. T. Choi, 2021

nb = length(bd); % number of boundaries

%% set up the outer boundary condition
bd_outer_id = 1;
bdy_index = bd{bd_outer_id};
bdy_length = sqrt((v(bdy_index,1) - v(bdy_index([2:end,1]),1)).^2 + ...
            (v(bdy_index,2) - v(bdy_index([2:end,1]),2)).^2);
partial_edge_sum = zeros(length(bdy_length),1);

% Arclength parameterization boundary constraint for the initial map
for i = 2:length(bdy_length)
    for j = 1:i-1
    partial_edge_sum(i) = partial_edge_sum(i) + bdy_length(j);
    end
end
theta = 2*pi.*partial_edge_sum/sum(bdy_length)+2*pi*0.01;
bdy = exp(theta*1i);

% % optimal rotation to align with the input mesh
corners = zeros(4,1);
[~,corners(1)] = min(abs(real(bdy) - min(v(:,1))) + abs(imag(bdy) - min(v(:,2))));
[~,corners(2)] = min(abs(real(bdy) - max(v(:,1))) + abs(imag(bdy) - min(v(:,2))));
[~,corners(3)] = min(abs(real(bdy) - max(v(:,1))) + abs(imag(bdy) - max(v(:,2))));
[~,corners(4)] = min(abs(real(bdy) - min(v(:,1))) + abs(imag(bdy) - max(v(:,2))));
[U, ~, ~] = Kabsch([real(bdy(corners)), imag(bdy(corners)),zeros(4,1)]', [v(bdy_index(corners),1:2),zeros(4,1)]');
target = (U(1:2,1:2)*[real(bdy), imag(bdy)]')';

target = [target(:,1)*r(bd_outer_id)+c(bd_outer_id,1), target(:,2)*r(bd_outer_id)+c(bd_outer_id,2)];

%% enforce all inner holes to be circular using the prescribed information
landmark = bd{bd_outer_id};

for i = setdiff(1:nb,bd_outer_id)

    % prescribed center
    cx = c(i,1);
    cy = c(i,2);
    % prescribed radius
    cr = r(i);
    
    bdy_index = bd{i};
    
    % project all boundary points onto the circle using arc-length parameterization
    bdyv = v(bd{i},:);
    bdyv(:,1) = bdyv(:,1) - cx;
    bdyv(:,2) = bdyv(:,2) - cy;
    [~,bdyv_id] = min(abs(angle(complex(bdyv(:,1),bdyv(:,2)))));
    
    bdy_index = bdy_index([bdyv_id:end, 1:(bdyv_id)-1]);
    
    bdy_length = sqrt((v(bdy_index,1) - v(bdy_index([2:end,1]),1)).^2 + ...
                (v(bdy_index,2) - v(bdy_index([2:end,1]),2)).^2);
    partial_edge_sum = zeros(length(bdy_length),1);

    % Arclength parameterization boundary constraint for the initial map
    for ii = 2:length(bdy_length)
        for jj = 1:ii-1
        partial_edge_sum(ii) = partial_edge_sum(ii) - bdy_length(jj);
        end
    end
    theta = 2*pi.*partial_edge_sum/sum(bdy_length);

    landmark = [landmark; bdy_index];
    bdy = cr*exp(theta*1i) + complex(cx,cy);
    target = [target; real(bdy), imag(bdy)];

end
%% obtain a bijective disk map with circular holes
mu = zeros(length(f),1); 

A = generalized_laplacian(v,f,mu); 
A(landmark,:) = 0;
A(landmark,landmark) = diag(ones(length(landmark),1));

bx = zeros(length(v),1); 
by = bx;
bx(landmark) = target(:,1);
by(landmark) = target(:,2);

% solve the generalized Laplace equation
map_x = A\bx;
map_y = A\by;
map = [map_x,map_y];
mu = beltrami_coefficient(v, f, map); 
 
IterationNumber = 0;
mu_bound = 0.95;
while max(abs(mu)) >= mu_bound
    mu(abs(mu)>=mu_bound) = mu_bound*mu(abs(mu)>=mu_bound)./abs(mu(abs(mu)>=mu_bound));
    
    A = generalized_laplacian(v,f,mu); 
    A(landmark,:) = 0;
    A(landmark,landmark) = diag(ones(length(landmark),1));

    bx = zeros(length(v),1); 
    by = bx;
    bx(landmark) = target(:,1);
    by(landmark) = target(:,2);

    % solve the generalized Laplace equation
    map_x = A\bx;
    map_y = A\by;
    map = [map_x,map_y];
    mu = beltrami_coefficient(v, f, map); 
    IterationNumber = IterationNumber + 1;
    if IterationNumber > 10
        break;
    end
end

% if max(abs(mu)) > 1
%     warning('There seem to be some flipped triangles.');
% end

end
