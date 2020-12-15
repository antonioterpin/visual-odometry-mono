function [p1, p, matches] = initializeTriangulation (...
    descriptors, descriptors1, keypoints, keypoints1)
%INITIALIZETRIANGULATION
    % Description goes here

    detector = HarrisDetectorBlock({}); % TODO obj property!!
    
    [matches, p1, p] = detector.getMatches(descriptors, descriptors1, keypoints, keypoints1);
    %TODO If a descriptor doesn't get a match, remove it from next
    %iteration of matchDescriptors
%     idx = matches(matches > 0);
    % [V;U]
    % Matched keypoints
%     keypointsMatched = keypoints(:, idx); % keypoints(:, matches > 0);
%     keypoints1Matched = keypoints1(:, matches > 0); % keypoints1(:, idx);
%     
%     p1 = flipud(keypoints1Matched);
%     p = flipud(keypointsMatched);
end