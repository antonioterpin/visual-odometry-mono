verboseDisp(obj.verbose, ...
    '\n\nProcessing frame %d\n=====================\n', ii);

[landmarks, landmarksIdx, keypoints] ...
    = obj.state.getObservations(ii - obj.nSkip);

[kpold, keypoints, keypointsLost, keep] =...
    obj.klt.KltTracker(inputHandler, ii, keypoints);

landmarks = landmarks(:, keep);
disp('landmarks kept by KLT')
sum(keep)

[R_CW, t_CW, inliers] = p3pRANSAC(keypoints, landmarks, K, ...
    obj.p3pRANSACIt, obj.p3pTolerance, obj.minInliers, obj.adaptive,...
    obj.verbose);
    %inliers = true(size(inliers)); %%%%%%%%%%%%%%%%% test
if ~isempty(R_CW) && ~isempty(t_CW)
    kpold = kpold(:, inliers);
    trackedKeypoints = keypoints(:, inliers);
    keypointsLost = [keypointsLost, keypoints(:, ~inliers)];
    landmarks = landmarks(:, inliers);
    
    disp('landmarks kept by RANSAC')
    sum(inliers)

    pose1Matrix = reshape(pose, [3,4]);
    poseMatrix = [R_CW, t_CW];
end

%TODO Generate keypointsNew and filter out doubles w.r.t. keypoints
needNewLandmarks = false;
if needNewLandmarks     %TODO add this bool
    %TODO keypoints = [keypoints, keypointsNew] where keypointsNew are
    %generated before the if statement for every iteration bu not used if
    %not inside the if
    
    landmarks = triangulationForKlt(K, kpold,...
        trackedKeypoints, pose1Matrix, poseMatrix, 4);
end

%Status stuff
image = inputHandler.getImage(ii);
trackedLandmarks = landmarks;

if ~isempty(R_CW) && ~isempty(t_CW)
    pose = [R_CW(:); t_CW(:)];
    lostKeypoints = keypointsLost;
    landmarksIdx = landmarksIdx(keep);
    landmarksIdx = landmarksIdx(inliers);
else
    pose = [];
end