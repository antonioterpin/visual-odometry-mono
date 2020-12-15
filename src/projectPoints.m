function [projections] = projectPoints (landmarks, K, D)
    %PROJECTPOINTS projects 3D landmarks on the image plane in pixel
    %coordinates given the intrisics parameters K and distorsion D
    %(optional parameter).
    %The function exploits the projection equation using normalized coordinates.
    %The function gets as inputs the [3 x N] landmarks matrix, arranged as
    %[X, Y, Z].
    
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