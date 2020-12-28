function landmarks = triangulationForKlt (K, keypoints1,...
    keypoints, pose1, pose, sampleSize)
    %TRIANGULATIONFORKLT
    
    assert(size(keypoints, 2) >= sampleSize);
    %TODO put this out of the function s.t. if not enough keypoints,
    %initialization.
    
    p1 = [keypoints1; ones(1, size(keypoints1, 2))];
    p = [keypoints; ones(1, size(keypoints, 2))];
    M1 = K * pose1;
    M = K * pose;
    landmarks = linearTriangulation(p1, p, M1, M);
    landmarks = landmarks(1:3, :);
end