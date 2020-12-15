function [R_C_W, t_C_W] = p3pRANSAC(keypoints, landmarks, K,...
    p3pIterations, p3pTolerance)
%keypoints Nx2, [U,V]
%landmarks Nx3

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