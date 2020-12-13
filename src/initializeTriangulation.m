function [p1, p, matches] = initializeTriangulation (...
    descriptors, descriptors1, matchLambda, keypoints, keypoints1)
%INITIALIZETRIANGULATION
    % Description goes here

    matches = matchDescriptors(descriptors, descriptors1, matchLambda);
    %TODO If a descriptor doesn't get a match, remove it from next
    %iteration of matchDescriptors
    idx = matches(matches > 0);
    % [V;U]
    % Matched keypoints
    keypointsMatched = keypoints(:, matches > 0);
    keypoints1Matched = keypoints1(:, idx);
    
    p1 = flipud(keypoints1Matched);
    p = flipud(keypointsMatched);
end