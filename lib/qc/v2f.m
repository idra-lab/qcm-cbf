function S = v2f(v,f)
% Input:
% v: nv x 2 or 3 vertex coordinates
% f: nf x 3 triangulations
%
% Output:
% S: nf x nv vertex-to-face conversion matrix

    nv = length(v); 
    nf = length(f);
    II = repmat((1:nf)',3,1);
    JJ = f(:);
    S = sparse(II,JJ,ones(length(JJ),1),nf,nv)./3;
end
