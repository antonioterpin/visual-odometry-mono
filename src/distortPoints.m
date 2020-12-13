function distorted = distortPoints (p, D)
    %DISTORTPOINTS Applies lens distorsion D to the keypoints in p [2xN]
    
    r2 = p(1, :).^2 + p(2, :).^2;
    x = p(1, :) .* (1 + D(1)*r2 + D(2) * r2.^2);
    y = p(2, :) .* (1 + D(1)*r2 + D(2) * r2.^2);
    distorted = [x; y];
end