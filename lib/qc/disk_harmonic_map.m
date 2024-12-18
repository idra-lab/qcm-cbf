function map = disk_harmonic_map(v,f)

if size(v,2) == 2
    v = [v, zeros(size(v,1),1)];
end

bd = meshboundaries(f);
nv = length(v);
bdy_index = bd{1};
% Remark: The above approach for getting the surface boundary may not work
% well in case the boundary contains vertices with valence 1
% In that case, use some other method to obtain the boundary first

bdy_length = sqrt((v(bdy_index,1) - v(bdy_index([2:end,1]),1)).^2 + ...
            (v(bdy_index,2) - v(bdy_index([2:end,1]),2)).^2 + ...
            (v(bdy_index,3) - v(bdy_index([2:end,1]),3)).^2);
partial_edge_sum = zeros(length(bdy_length),1);

% Arclength parameterization boundary constraint for the initial map
for i = 2:length(bdy_length)
    for j = 1:i-1
    partial_edge_sum(i) = partial_edge_sum(i) + bdy_length(j);
    end
end
theta = 2*pi.*partial_edge_sum/sum(bdy_length)+2*pi*0.01;
bdy = exp(theta*1i);

% Disk harmonic map
M = cotangent_laplacian(v,f);

[mrow,mcol,mval] = find(M(bdy_index,:));
M = M - sparse(bdy_index(mrow),mcol,mval,nv, nv) + ...
        sparse(bdy_index,bdy_index,ones(length(bdy_index),1),nv, nv);
c = zeros(nv,1); c(bdy_index) = bdy;
z = M \ c;
map = [real(z),imag(z)]; 

if sum(sum(isnan(map))) ~= 0
    % use tutte embedding instead
    map = tutte_map(v,f,bdy_index,bdy); 
end