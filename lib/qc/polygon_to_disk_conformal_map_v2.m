function map = polygon_to_disk_conformal_map_v2(p,v)

% Conformally map a polygon to the unit disk, with the exterior of the polygon 
% mapped to the exterior of the disk. Based on [1] with certain modifications.
%
% Input:
% p: np x 2 coordinates of the polygon 
% v: nv x 2 coordinates of all points to be mapped (can include both the
% exterior points and the polygon points)
%
% Output:
% map: nv x 2 coordinates of the mapped points
%
% References:
% [1] G. P. T. Choi, Y. Leung-Liu, X. Gu, L. M. Lui,
%    Parallelizable global conformal parameterization of simply-connected surfaces via partial welding.
%    SIAM J. Imaging Sci., 13(3), 1049-1083, 2020.

%%
% ensure that the polygon vertices are in anticlockwise order
if ispolycw(p(:,1),p(:,2))
    p = flipud(p);
end

zp = complex(p(:,1),p(:,2));
zv = complex(v(:,1),v(:,2));
n = size(zp, 1);
w = [zp; Inf+Inf*1i; zv; 10000]; % add an auxilliary point of infinity and an auxillary point for rotation

% apply a series of conformal maps

% map everything to the right half-plane
w = step0(w, w(1), w(2));

% map polygon vertices to the imaginary axis one by one
for j = 3:n
    w = step1(w, w(j), -1i);
end

% map everything to the lower half-plane
w = step2(w, w(1));

% map the real axis to the unit circle
w = step3(w, 1i);

% apply an inversion to map everything to the interior of the disk
w = 1./conj(w);

% apply a mobius transformation to shift the point of infinity to the origin
w = (w-w(n+1))./(1-conj(w(n+1))*w);

% apply an inversion to map everything to the exterior of the disk
w = 1./conj(w);

% add an extra rotation
theta = -angle(w(end));
w = exp(1i*theta)*w;

map = [real(w(n+2:end)),imag(w(n+2:end))];

end

function w = step0(z, p, q)
w = sqrt((z-q)./(z-p));
w(isinf(z)) = 1;
end

function w = step1(z, p, m)
if p == 0
    p = eps;
end
c = real(p) / abs(p)^2;
d = imag(p) / abs(p)^2;
t = c * z ./ (1 + 1i*d*z);
t(isinf(z)) = c / (1i*d);
w = sqrt(t.^2-1);
% resolve branching difficulty
k = imag(w).*imag(t) < 0;
w(k) = -w(k);
w(z == 0) = m;
w(z == p) = 0;
end

function w = step2(z, p)
w = (z./(1-z/p)).^2;
w(isinf(z)) = (-p).^2;
end

function w = step3(z, p)
w = (z-p)./(z-conj(p));
w(isinf(z)) = 1;
end