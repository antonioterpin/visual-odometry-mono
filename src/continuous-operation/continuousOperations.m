function [pose, keypointsMatched, landmarksMatched, ...
    lastKeypoints, lastDescriptors, keypoints1Matched] =...
    continuousOperations(K, descriptors, keypoints, descriptors1, keypoints1, landmarks1, lastKeypoints,...
    lastDescriptors)
    %CONTINUOUSOPERATIONS
    
    %state1 is [12xN] state matrix (to reshape)
    % TODO THESE SHOULD BECOME PROPERTIES!
    global p3pRANSACIt p3pTolerance detector;
    
    [matches, keypoints1Matched, keypointsMatched] = detector.getMatches(...
        descriptors1, descriptors, keypoints1, keypoints);
    landmarksMatched = landmarks1(:, matches(matches > 0) );
    keypointsMatched = keypointsMatched(1:2, :);
    
    %% Find R and t with P3P + Ransac
    [R_IC, t_IC] = p3pRANSAC(keypointsMatched(1:2, :)', landmarksMatched', K,...
        p3pRANSACIt, p3pTolerance);
    pose = [R_IC(:); t_IC(:)];  %12x1 pose vector
end