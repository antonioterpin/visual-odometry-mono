function [newState] = triangulationRANSAC (state, K, keypoints1,...
    keypoints, pose1, sample, tolerance, iterations)
%TRIANGULATIONRANSAC triangulates new 3D landmarks starting from two
    %arrays of keypoints, one of the current image and the other of the
    %previous one. The function uses the Eight Point and the RANSAC
    %algorithms. The Eigth Point algorithm allows to extrapolate the
    %Fundamental (or Essential) matrix given two sets of keypoints.
    %However, the algorithm needs some filtering in order to distinguish
    %between outliers and inliers and this is done using the RANSAC
    %algorithm. It chooses datapoints from a given set - in this case the
    %two homogeneous keypoints arrays - and through numerous iterations
    %distinguishes the inliers from the outliers.
    %The function selects the inliers that better fit the model and
    %computes the most likely Fundamental (or Essential) matrix. Then, it
    %extrapolates from it the rotation matrix R and the translation vector
    %t. In this way it can linearly triangulate new landmarks. These
    %landmarks are then put in the output variable newState together with
    %the corresponding keypoints.
    %
    %Inputs:
    %   state is an array [5 x N] where the first two rows are the keypoinys
    %       [U,V] and the last three rows the landmarks [X,Y,Z].
    %   K is the matrix of intrinsic parameters [3 x 3]
    %   keypoints1 is the matrix of previous keypoints, to be triangulated.
    %       [U, V], [2 x N]
    %   keypoints is the matrix of current keypoints, to be triangulated.
    %       [U, V], [2 x N]
    %   pose1 is the array [R | t], [3 x 4] that combines the rotation
    %       matrix together with the translation vector
    %   sample is the minimum number of previous keypoints to start the
    %       triangulation
    %   tolerance is the maximum allowed distance of an inlier with respect
    %       to the epipolar line
    %   iterations is the number of RANSAC iterations
    %
    %Outputs:
    %   newState is the matrix [5 x N] where the first two rows are the new
    %       triangulated keypoints and the last three rows the new
    %       triangulated landmarks.
    
if( size(keypoints1, 2) >= sample)
    p1 = [keypoints1; ones(1, size(keypoints1, 2))];
    p = [keypoints; ones(1, size(keypoints, 2))];
    [T, ~, ~] = estimateRelativePose(p1, p, K, K);
    T1 = [pose1; 0,0,0,1];
    M1 = K * pose1;
    P = triangulateFromPose( p1, p, T, K, K, T1);
    F = [];
    [F, ~] = RANSAC(...
        @(data, otherParams) estimateFundamentalMatrix(data(1:3,:), data(4:6,:),true), ...
        @(candidate, data, otherParams) errorMetric(data(1:3,:), data(4:6,:), candidate(:)),...
        8, [p; p1], iterations, [], false);
    if( isempty(F) )
        F = estimateFundamentalMatrix(p1,p,true);
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
    [R_IC, t_IC] = disambiguatePose(R, u, p1(:,inliers), p(:,inliers), K);
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
            [keypoints; P(1:3, :)]];
        disp('NEW STATE ADDED WITH SUCCESS!!')
    end
else
    newState = state;   %if not possible to run triangulation, skip
end

end