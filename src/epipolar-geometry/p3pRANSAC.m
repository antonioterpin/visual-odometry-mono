function [R_C_W, t_C_W] = p3pRANSAC(keypoints, landmarks, K,...
    p3pIterations, p3pTolerance)
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

maxInliers = 0;
if(size(landmarks, 1) > 2)
    for i = 1 : p3pIterations
        [landmarkSample, idx] = datasample(landmarks, 3, 1, 'Replace', false);
        keypointSample = keypoints(idx, :);
        keypointNormHom = K \ [keypointSample, ones(3, 1)]';
        for j = 1 : 3
            keypointNormHom(:, j) = keypointNormHom(:, j) / ...
                norm(keypointNormHom(:, j), 2);
        end
        poses = p3p(landmarkSample', keypointNormHom); %WE COULD PERSONALIZE THIS P3P FUNCTION
        inliers = zeros(1, size(keypoints, 1));
        for j = 0 : 1
            guessR_CI = real(poses(:, (2 + j * 4) : (4 + j * 4)));
            guesst_CI = real(poses(:, (1 + j * 4)));
            R_IC = guessR_CI';
            t_IC = -guessR_CI' * guesst_CI;
            reprojection = projectPoints(...
                R_IC * landmarks' + repmat(t_IC, [1, size(landmarks, 1)]), K);
            difference = keypoints' - reprojection;
            errors = sum(difference.^2, 1);
            inliersGuess = errors < p3pTolerance^2;
            if( nnz(inliersGuess) > nnz(inliers) )
                inliers = inliersGuess;
                rightR_IC = R_IC;
                rightt_IC = t_IC;
            end
        end
        if( nnz(inliers) > maxInliers && nnz(inliers) >= 6)
            maxInliers = nnz(inliers);
            R_C_W = rightR_IC;
            t_C_W = rightt_IC;
        end
    end
    if maxInliers == 0
        disp('Impossible to create new Pose');

         R_C_W = [];
         t_C_W = [];
         return
    end
else
    disp('Impossible to create new Pose');

     R_C_W = [];
     t_C_W = [];
     return
end
end