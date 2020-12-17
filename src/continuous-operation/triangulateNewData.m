function [triangulatedKeypoints, triangulatedLandmarks, triangulatedDescriptors, ...
    triangulationCandidatesKeypoints, triangulationCandidatesDescriptors, sizeHistory] ...
    = triangulateNewData(K, keypoints, descriptors, keypointsMatched, ...
    triangulationCandidatesKeypoints, triangulationCandidatesDescriptors, sizeHistory, currPose)

global newPointsTolerance triangulationSample newPointsRANSACIt detector;
triangulatedKeypoints = []; triangulatedLandmarks = []; triangulatedDescriptors = [];

% Find unmatched keypoints
% Retrieves the Descriptors and Keypoints without a Landmark-match
unMatchedIndices = ~ismember(keypoints', keypointsMatched','rows'); %get index of unmatched keypoints
% [V;U]
keypointsTri = keypoints(:,unMatchedIndices);   % unmatched keypoints
descriptorsTri = descriptors(:,unMatchedIndices);
%OSS We get unmatched keypoints because the matched ones already have
%their correspondent landmark in the state1 vector

% Find new landmarks with triangulation
unmatchedPostTriIdx = ones(size(keypoints,1),1);
if(~isempty(triangulationCandidatesKeypoints) && ~isempty(triangulationCandidatesDescriptors)) %robustness check
    [matchIndices, p1, p] = detector.getMatches(...
        descriptorsTri, triangulationCandidatesDescriptors, keypointsTri, triangulationCandidatesKeypoints);
    lastPose = reshape(currPose, 3, 4);
    triangulatedDescriptors = descriptorsTri(:, matchIndices(matchIndices > 0)); % TODO MIGHT BE matchIndices > 0
    
    if size(p1,2) >= triangulationSample
        [triangulatedKeypoints, triangulatedLandmarks, inliers] = triangulationRANSAC(K,...
            p1, p, lastPose, triangulationSample, newPointsTolerance,...
            newPointsRANSACIt);
        triangulatedDescriptors = triangulatedDescriptors(:,inliers);
        triangulatedLandmarks = triangulatedLandmarks(1:3,:);
    end
    
    unmatchedPostTriIdx = ~ismember(keypoints', p(1:2, :)','rows'); %Indexes of keypoints unmatched after triangulation
end

% Save all unmatched keypoints and descriptors for next triangulations
% unmatchedPostTriIdx= ~ismember(keypoints', p(1:2, :)','rows'); %Indexes of keypoints unmatched after triangulation
unmatchedKeypoints = keypoints(:,unmatchedPostTriIdx);

unmatchedDescriptors = descriptors(:, unmatchedPostTriIdx);

[sizeHistory, triangulationCandidatesKeypoints, triangulationCandidatesDescriptors] =...
    maintainTriangulationHistory (sizeHistory, triangulationCandidatesKeypoints, triangulationCandidatesDescriptors, ...
    unmatchedKeypoints, unmatchedDescriptors);

end