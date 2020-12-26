verboseDisp(obj.verbose, ...
    '\n\nProcessing frame %d\n=====================\n', ii);

[landmarks, landmarksIdx, keypoints] ...
    = obj.state.getObservations(ii - obj.nSkip);

prevImage = inputHandler.getImage(ii - obj.nSkip);
descriptors = obj.coBlock.Detector.describeKeypoints(prevImage, keypoints);

image = inputHandler.getImage(ii);

[obj.coBlock, trackedKeypoints, ~, trackedLandmarks, ...
    R_CW, t_CW, tracked_mask, unmatchedKeypoints, unmatchedDescriptors] ...
    = obj.coBlock.localize(descriptors, landmarks, image);

if isempty(R_CW) || isempty(t_CW)
    pose = [];
else
    pose = [R_CW(:); t_CW(:)];
    lostKeypoints = keypoints(:, ~tracked_mask);
    landmarksIdx = landmarksIdx(tracked_mask);
end
