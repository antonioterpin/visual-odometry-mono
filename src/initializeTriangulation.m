function [p1, p, matches] = initializeTriangulation (...
    descriptors, descriptors1, matchLambda, keypoints, keypoints1)
%INITIALIZETRIANGULATION generates the keypoint matching
%between two sets of data. It takes as input the current
%descriptors and keypoints - descriptors, keypoints - and the ones deriving
%from the previous iteration - descriptors1, keypoints1 - together with a
%parameter - matchLambda - that is a parameter useful to tune the matching
%threshold. The keypoints in the input are passed as [2 x N] matrices, with
%the first row corresponding to the image coordinate V and the second to U.
%The descriptors in the input are [M x N] matrices, where N must be the
%same as the second dimension of the keypoint matrix, since every
%descriptor has to have a direct correspondance to a keypoint. The first
%dimension of the descriptos is free and is based on personal choice and
%different euristics.
%
%The function returns the vector matches, that indicates the
%correspondences between keypoints and keypoints1 based on the descriptors'
%similarities. The indexes of the vector matching correspond to the
%elements of the vector keypoints. Thus, for each index i in matches, there
%is a value in the vector corresponding to the index of the corresponding
%keypoint in the vector keypoints1. If the i-th keypoint has no matches,
%the returned value in the corresponding position is zero.
%The function also returns the matrices p and p1. These are respectively
%the vectors keypoints and keypoints1 filtered based on the matching. p and
%p1 are returned as [2 x L] matrices where the first row corresponds to the
%dimension U and the second to V.
    matches = matchDescriptors(descriptors, descriptors1, matchLambda);
    idx = matches(matches > 0);
    % [V;U]
    % Matched keypoints
    keypointsMatched = keypoints(:, matches > 0);
    keypoints1Matched = keypoints1(:, idx);
    
    p1 = flipud(keypoints1Matched);
    p = flipud(keypointsMatched);
end