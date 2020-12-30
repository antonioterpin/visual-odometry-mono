function W = getSimWarp(dx, dy, alpha_deg, lambda)
% alpha given in degrees, as indicated
    
    alpha = deg2rad(alpha_deg);
    c = cos(alpha);
    s = sin(alpha);

    W = lambda * [c, -s, dx; s, c, dy];

end
