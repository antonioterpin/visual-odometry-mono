function [newState] = triangulationRANSAC (state, K, keypoints1,...
    keypoints, pose1, sample, tolerance, iterations)
    %the function has to be put into a for cycle where the data to be
    %processed to get each new landmark are passed in
    
    %keypoints1 are the keypoints used for triangulation at the previous step ,that
    %have a match in the new one. [U,V], [2xN]
    
    %keypoints are the keypoints in the current step that have a match. [U,V], [2xN]
    
    %pose1 is the [3x4] matrix containing the last available pose [R|t]
    
    
if( size(keypoints1, 2) >= sample)
    p1 = [keypoints1; ones(1, size(keypoints1, 2))];
    p = [keypoints; ones(1, size(keypoints, 2))];
    [T, ~, ~] = estimateRelativePose(p1, p, K, K);
    T1 = [pose1; 0,0,0,1];
    M1 = K * pose1;
    P = triangulateFromPose( p1, p, T, K, K, T1);
    F = [];
    [F, ~] = RANSAC(...
        @(data, otherParams) fundamentalEightPoint_normalized(data(1:3,:), data(4:6,:)), ...
        @(candidate, data, otherParams) errorMetric(data(1:3,:), data(4:6,:), candidate(:)),...
        iterations, ...
        0.99, 8, [p; p1], []);
    if( isempty(F) )
        F = fundamentalEightPoint_normalized( p1, p );
        %TODO Is epipolar line distance the metric we want?
        d = epipolarLineDistance( F, p1, p );
    else
        %TODO Is epipolar line distance the metric we want?
        d = epipolarLineDistance( F, p1, p );
    end
    inliers = find( d < tolerance );    %TODO Experiment changing tolerance for tuning # new landmarks added
    if( size(p1(:, inliers), 2) >= 8 )
        E = estimateEssentialMatrix(p1(:, inliers), p(:, inliers), K, K);
    else
        E = estimateEssentialMatrix(p1, p, K, K);
        %TODO Introduce another way of computing R,t instead of using
        %"outliers"?
    end
    [R, u] = decomposeEssentialMatrix( E );
    [R_IC, t_IC] = disambiguateRelativePose( R, u, p1(:, inliers),...
        p(:, inliers), K * eye(3,4), K);
    T = [R_IC, t_IC; 0,0,0,1];
    M = T * T1;
    M = K * M(1:3, 1:4);
    P = linearTriangulation(p1(:, inliers), p(:, inliers), M1, M);
    keypoints = keypoints(:, inliers);
    
    disp([num2str(size(P, 2)) ' newly triangulated points']);

    % Return appropriate output w.r.t. situation (robustness)
    if (isempty(keypoints) && ~isempty(state))
        newState = state;
    elseif (isempty(keypoints) && isempty(state))
        newState = [];
    else
        newState = [state,...
            [flipud(keypoints); P(1:3, :)]];
        disp('NEW STATE ADDED WITH SUCCESS!!')
    end
else
    newState = state;   %if not possible to run triangulation, skip
end

end