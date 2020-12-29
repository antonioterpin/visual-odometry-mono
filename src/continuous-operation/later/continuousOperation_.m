verboseDisp(obj.verbose, ...
    '\n\nProcessing frame %d\n=====================\n', frameIdx);

[landmarks, landmarksIdx, keypoints] ...
    = obj.state.getObservations(prevFrameIdx);

prevImage = inputHandler.getImage(prevFrameIdx);
descriptors = obj.coBlock.Detector.describeKeypoints(prevImage, keypoints);

image = inputHandler.getImage(frameIdx);

[trackedKeypoints, ~, trackedLandmarks, ...
    R_CW, t_CW, tracked_mask, unmatchedKeypoints, unmatchedDescriptors] ...
    = obj.coBlock.localize(descriptors, landmarks, image);

if isempty(R_CW) || isempty(t_CW)
    pose = [];
else
    pose = [R_CW(:); t_CW(:)];
    lostKeypoints = keypoints(:, ~tracked_mask);
    landmarksIdx = landmarksIdx(tracked_mask);
end
