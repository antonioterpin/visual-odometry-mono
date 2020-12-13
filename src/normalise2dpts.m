function [pN, T] = normalise2dpts (p)
    %NORMALISE2DPTS Takes [3xN] array of 2D homogeneous image points,
    %applies normalization and translation of their centroid to the origin
    %s.t. their average distance from the origin is sqrt(2).
    
    arguments
        p (3,:)
    end
    % pN is [3xN] array of transformed 2D homogeneous coordinates
    % T is [3x3] transformation matrix s.t. pN = T * p
    
    mu = mean(p(1:2, :), 2);
    sigma = mean(sqrt(sum((p(1:2, :) - repmat(mu, 1, length(p))).^2)));
    s = sqrt(2) / sigma;
    T = [[s, 0, -s*mu(1)];
        [0, s, -s*mu(2)];
        [0, 0, 1]];
    pN = T * p;
end