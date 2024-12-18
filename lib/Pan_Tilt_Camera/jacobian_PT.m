function J = jacobian_PT(q, l)
    l0 = l(1);
    l1 = l(2);
    l2 = l(3);

    c1 = cos(q(1));
    s1 = sin(q(1));
    c2 = cos(q(2));
    s2 = sin(q(2));
    
    J = [        0,     l2*c2; ...
          l2*c1*c2, -l2*s1*s2; ...
          l2*s1*c2,  l2*c1*s2; ...
                 1,         0; ...
                 0,       -c1; ...
                 0,       -s1 ];
end