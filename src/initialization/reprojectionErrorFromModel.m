function [isInlier, error] = reprojectionErrorFromModel(candidate, data, otherParams)
    % Extract data
    T_21 = candidate;
    p1 = data(1:3,:);
    p2 = data(4:6,:);
    K = reshape(otherParams(1:9), 3, 3);
    errorTh = otherParams(26);
    maxDist = otherParams(27);
    T_1W = reshape(otherParams(10:25), 4, 4);
    T_2W = T_21(1:3,:) * T_1W;
    
    % Triangulation
    landmarks = triangulateFromPose(p1, p2, T_21, K, K, T_1W);
    
    distances = vecnorm(landmarks(1:3,:) + T_2W(1:3,1:3).' * T_2W(1:3,4), 2, 1);
    
    % Reprojection of valid landmarks (world points)
    error = reprojectionError(...
        landmarks(1:3,:), p2(1:2,:), K, T_2W(1:3,1:3), T_2W(1:3,4));
    isInlier = error < errorTh & distances < maxDist;
end

