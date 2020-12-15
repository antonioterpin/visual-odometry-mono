function [projections] = projectPoints (landmarks, K, D)
    %PROJECTPOINTS Projects 3D landmarks on the image plane in pixel
    %coordinates given the intrisics parameters K and distorsion D
    %(optional parameter).
    
    %landmarks [3xN]
    %K [3x3]
    
    if nargin <= 2
        D = zeros(4, 1);
    end
    % Normalized coordinates
    x = landmarks(1, :) ./ landmarks(3, :);
    y = landmarks(2, :) ./ landmarks(3, :);
    % Distorsion
    distorted = distortPoints([x; y], D);
    
    % Pixel coordinates
    projections = K * [distorted(1, :); distorted(2, :);...
        ones(1, size(landmarks, 2))];
    projections = projections(1:2, :);
end