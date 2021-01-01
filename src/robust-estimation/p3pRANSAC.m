function [R_CW, t_CW, inliers] = p3pRANSAC(keypoints, landmarks, K,...
    p3pIterations, p3pTolerance, minInliers, adaptive, verbose)
% TODO fix documentation
% keypoints 2xN [U, V]
% landmarks 3xN
%
%
%P3PRANSAC computes the rotation matrix R_C_W that converts the camera
%frame points into the (inertial) world frame points, and the translation 
%vector t_C_W pointing to the camera frame and starting in the world frame.
%The function performs the task combining the P3P and the RANSAC
%algorithms. P3P is the algorithm that allows to go from three
%keypoints-landmarks associations to four possible transformation matrices
%of the form [R|t]. It is possible to disambiguate the poses with a fourth
%point. However, the algorithm may be influenced by some outlier in the
%data and for this reason the model-fitting random algorithm RANSAC is
%deployed. It randomly picks, for every iteration, a sample of data from
%the ones available in the input landmark matrix and ripetitively computes
%the P3P algorithm, saving as best guess the one that minimizes a certain
%error - in this case the reprojection error. In the end, once the most
%likely pose has been found, the values are returned.
%
%The function takes as input the keypoints matrix, a [N x 2] matrix,
%ordered as [U, V], that represent the keypoints in the image plane matched
%with some landmarks in the 3D space. These landmarks are also passed as
%inputs in a [N x 3] matrix organized as [X, Y, Z] coordinates.
%The function takes also as input the matrix of intrinsic parameters K and
%the values p3pIterations, specifying the number of RANSAC iterations, and
%p3pTolerance, specifying the tuning value for the reprojection error.
%
% TODO

arguments
    keypoints (2,:)
    landmarks (3,:)
    K (3,3)
    p3pIterations = 1000
    p3pTolerance = 10
    minInliers = 30
    adaptive = 0.95
    verbose = true
end

% keypoints = flipud(keypoints);

% TODO argument validation
assert(size(landmarks, 2) > 2, 'p3pRANSAC requires at least 3 points.');

normalizedBearings = normalize(K \ [keypoints; ones(1, size(keypoints,2))], 'norm');
data = [normalizedBearings; landmarks; keypoints];
otherParams = [K(:); p3pTolerance^2];

% We use p3p + RANSAC to get the inliers
[model, inliers] = RANSAC(@modelFromSample, @errorMetric, 3, data, ...
    p3pIterations, otherParams, verbose, adaptive);

model = model(:,:,inliers(1));
inliers = inliers(2:end) > 0;

verboseDisp(verbose,...
        'Found %d inliers to estimate pose. Estimated outlier ratio: %.3f%%\n',... 
        [nnz(inliers), (1 - nnz(inliers) / numel(inliers)) * 100]);

if nnz(inliers) < minInliers
    verboseDisp(verbose, 'Unable to localize');
    R_CW = [];
    t_CW = [];
    inliers = false(1, size(keypoints,2));
else
%     % To effectively estimate the pose, we use DLT
%     M_CW = estimatePoseDLT(keypoints(:, inliers).', landmarks(:, inliers).', K);
%     R_CW = M_CW(:, 1:3);
%     t_CW = M_CW(:, end);

    R_WC = model(:, 2:4);
	t_WC = model(:, 1);
    R_CW = R_WC.';
    t_CW = -R_WC.'*t_WC;
end

end

function model = modelFromSample(sample, ~)
    normBearingsSample = sample(1:3,:);
    landmarksSample = sample(4:6,:);
    
    % 3 points are used to get 4 possible solutions
    solutions = p3p(landmarksSample, normBearingsSample);
    model = reshape(real(solutions), 3, 4, 4);
end

function [isInlier, error] = errorMetric(candidate, data, otherParams)
    landmarks = data(4:6, :);
    keypoints = data(7:8, :);
    
    K = reshape(otherParams(1:9), 3, 3);
    errorTolerance = otherParams(10);

    maxNInliers = -1;
    correctModel = 0;
    for j = 1:4
        R_WC_j = candidate(:, 2:4, j);
        t_WC_j = candidate(:, 1, j);
        
        R_CW_j = R_WC_j.';
        t_CW_j = - R_WC_j.' * t_WC_j;
        
        % the fourth point is used to disambiguate
        errors = reprojectionError(landmarks, keypoints, K, R_CW_j, t_CW_j);
        inliers = errors < errorTolerance;
        nInliers = nnz(inliers);
        if(maxNInliers < nInliers)
            maxNInliers = nInliers;
            isInlier = inliers;
            error = errors;
            correctModel = j;
        end
    end
    
    % a bit of a hack
    isInlier = [correctModel, isInlier];
end