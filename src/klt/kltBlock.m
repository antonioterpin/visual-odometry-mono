verboseDisp(obj.verbose, ...
    '\n\nProcessing frame %d\n=====================\n', ii);
if obj.justInitialized
    [landmarks, ~, ~] ...
    = obj.state.getObservations(ii - obj.nSkip);
else
    landmarks = trackedLandmarks;
end

[~, landmarksIdx, keypoints] ...
    = obj.state.getObservations(ii - obj.nSkip);

keypoints = keypoints / 4;

[kpold, keypoints, keypointsLost, keep] =...
    obj.klt.KltTracker(inputHandler, ii, keypoints);
keypointsLost = keypointsLost * 4;
keypoints = keypoints * 4;
kpold = kpold * 4;
landmarks = landmarks(:, keep);

%     1. P3P using old landmarks
[R_CW, t_CW, inliers] = p3pRANSAC(keypoints, landmarks, K,...
    obj.p3pRANSACIt, obj.p3pTolerance, obj.minInliers, obj.adaptive,...
    obj.verbose);
%       2. Triangulation to generate new landmarks
poseKlt = reshape(pose, [3, 4]);
poseKltNew = [R_CW, t_CW];    %new pose
if ~isempty(R_CW) && ~isempty(t_CW)
    [keypoints, landmarks] = triangulationForKlt (K, kpold,...
        keypoints, poseKlt, poseKltNew, 4);

    image = inputHandler.getImage(ii);

    %errorMetrics stuff
    trackedKeypoints = keypoints;
    trackedLandmarks = landmarks;

    pose = [R_CW(:); t_CW(:)];
    lostKeypoints = keypointsLost;
    landmarksIdx = landmarksIdx(keep);
else
    pose = [];
    trackedKeypoints = keypoints;
end