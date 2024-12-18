function [bds, outer] = meshboundaries(f)

%  MESHBOUNDARIES Mesh Boundaries
%     [bds, n] = MESHBOUNDARIES(f) extracts boundaries of mesh into cells

%% this version is a little bit faster
% g = sparse(f, f(:, [2, 3, 1]), 1);
% [ii, jj, ~] = find(g+g' == 1);
% h = digraph(ii, jj);
% bds = {};
% visited = true(max(f(:)), 1);
% visited(ii) = false;
% n = 0;
% while 1
%   kk = find(~visited);
%   if isempty(kk)
%     break
%   end
%   n = n+1;
%   bds{n} = dfsearch(h, kk(1));
%   visited(bds{n}) = true;
% end
%% this version is older but works
nv = max(f(:));
v = zeros(nv, 2);
tr = triangulation(f, v);
fe = tr.freeBoundary;
if isempty(fe)
  bds = {};
  outer = [];
  return;
end
[~, order] = sort(fe(:, 1));
fe = fe(order, :); % reordered boundary edges
ex = zeros(nv, 1); 
vs = false(nv, 1); 
nfe = size(fe, 1);
for i = 1 : nfe
  ex(fe(i, 1)) = fe(i, 2); % table of second edge vertex
end
n = 0;
for i = unique(fe(:))' % for each boundary vertex
  if ~vs(i) 
    n = n + 1; % record boundary components
    [bds{n}, vs] = dfs(ex, vs, i);
  end
end
%% 
m = zeros(n, 1);
for i = 1 : n
  m(i) = size(bds{i}, 1);
end
[~, id] = sort(m, 'descend');
bds = bds(id);
outer = bds{1};

function [bd, vs] = dfs(ex, vs, i)
vs(i) = 1; % marked
bd = i;
if ~vs(ex(i)) % if second vertex hasn't been marked
  [bd, vs] = dfs(ex, vs, ex(i)); % mark second vertex
  bd = [i; bd]; % increase bd
end