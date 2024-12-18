function T = fk_PT(q, l)
    l0 = l(1);
    l1 = l(2);
    l2 = l(3);

    c1 = cos(q(1));
    s1 = sin(q(1));
    c2 = cos(q(2));
    s2 = sin(q(2));

    x = l1 + l2*s2;
    y = l2*s1*c2;
    z = l0 - l2*c1*c2;

    T = [  0,     c2,     s2, x; ...
          c1, -s1*s2,  s1*c2, y; ...
          s1,  c1*s2, -c1*c2, z;
           0,      0,      0, 1 ];
end