function map = polygonal_to_ball_qc_map_2d(v,f)
% Compute a quasi-conformal map of a multiply-connected open mesh onto 
% the unit disk with circular holes
%
% Input:
% v: nv x 2 vertex coordinates of a simply-connected open triangle mesh
% f: nf x 3 triangulations of a simply-connected open triangle mesh
% 
% Output:
% map: nv x 2 vertex coordinates of the disk with circular holes
%
% Written by Gary P. T. Choi, 2020

bd = meshboundaries(f);
nb = length(bd); % number of boundaries
bd_list = 1:nb;

% find the outermost boundary
bd_outer_id = 1;
bd_outer = bd{bd_outer_id};
bd_outer_length = sum(sqrt(sum((v(bd_outer,:) - v(bd_outer([2:end,1]),:)).^2,2)));
for i = 2:nb
    bd_next = bd{i};
    bd_next_length = sum(sqrt(sum((v(bd_next,:) - v(bd_next([2:end,1]),:)).^2,2)));
    if bd_next_length > bd_outer_length
        bd_outer_id = i;
        bd_outer_length = bd_next_length;
    end
end

nv = length(v);

v_filled = v;
f_filled = f;
count = 1;
for i = setdiff(bd_list,bd_outer_id)
    % fill all holes 
    centroid = sum(v(bd{i},:),1)/length(bd{i});
    v_filled = [v_filled; centroid];
    f_filled = [f_filled; bd{i}([2:end,1]), bd{i}, (nv + count)*ones(length(bd{i}),1)];
    count = count + 1;
end

%% compute a disk harmonic map
map_harmonic_filled = disk_harmonic_map(v_filled,f_filled);
map_harmonic = map_harmonic_filled(1:nv,:);

% optimal rotation to align the disk with the input mesh
corners = zeros(4,1);
[~,corners(1)] = min(abs(v(:,1) - min(v(:,1))) + abs(v(:,2) - min(v(:,2))));
[~,corners(2)] = min(abs(v(:,1) - max(v(:,1))) + abs(v(:,2) - min(v(:,2))));
[~,corners(3)] = min(abs(v(:,1) - max(v(:,1))) + abs(v(:,2) - max(v(:,2))));
[~,corners(4)] = min(abs(v(:,1) - min(v(:,1))) + abs(v(:,2) - max(v(:,2))));
[U, ~, ~] = Kabsch([map_harmonic(corners,1:2),zeros(4,1)]', [v(corners,1:2),zeros(4,1)]');
map_harmonic = (U(1:2,1:2)*map_harmonic')';

%% enforce all holes to be circular
landmark = bd{bd_outer_id};
target = map_harmonic(bd{bd_outer_id},:);
fa = sum(face_area(f,v));

for i = setdiff(bd_list,bd_outer_id)

    cx = mean(map_harmonic(bd{i},1));
    cy = mean(map_harmonic(bd{i},2));
    bdy_index = bd{i};
    
    % project all boundary points onto a circle
    bdyv = map_harmonic(bd{i},:);
    bdyv(:,1) = bdyv(:,1) - cx;
    bdyv(:,2) = bdyv(:,2) - cy;
    [~,bdyv_id] = min(abs(angle(complex(bdyv(:,1),bdyv(:,2)))));
    
    bdy_index = bdy_index([bdyv_id:end, 1:(bdyv_id)-1]);
    
    bdy_length = sqrt((v(bdy_index,1) - v(bdy_index([2:end,1]),1)).^2 + ...
                (v(bdy_index,2) - v(bdy_index([2:end,1]),2)).^2);
    partial_edge_sum = zeros(length(bdy_length),1);

    fa_holei = polyarea(v(bdy_index,1),v(bdy_index,2));
    
    cr = sqrt(fa_holei/fa);
    % Arclength parameterized boundary constraint
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
    
%% obtain a disk quasiconformal map with circular holes

mu = beltrami_coefficient(v, f, map_harmonic); 

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
mu_bound = 0.98;
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

if max(abs(mu)) > 1
    warning('There seem to be some flipped triangles.');
end

end
