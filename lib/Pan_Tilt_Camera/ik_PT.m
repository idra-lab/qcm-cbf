function q = ik_PT(p, l)
    l0 = l(1);
    l1 = l(2);
    l2 = l(3);
    
    x = p(1);
    y = p(2);
    z = p(3);
    
    lb = (y^2 + (l0-z)^2); % / l2^2;
    la = sqrt(lb);
    
    q(1) = atan2(y, l0-z);
    q(2) = atan2(x-l1, la);
    q(3) = atan2(x-l1, -la);
% 
%     q(1) = atan2(y, l0-z);
%     q(2) = atan2((x-l1)/l2, la);
%     q(3) = atan2((x-l1)/l2, -la);
    
    q = q(:);
end