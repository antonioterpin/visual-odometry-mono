verboseDisp(obj.verbose, ...
    '\n\nProcessing frame %d\n=====================\n', ii);

[landmarks, landmarksIdx, keypoints] ...
    = obj.state.getObservations(ii - obj.nSkip);

[kpold, keypoints, keypointsLost, keep] =...
    obj.klt.KltTracker(inputHandler, ii, keypoints);

landmarks = landmarks(:, keep);

disp('landmarks kept by KLT')
sum(keep)

if size(keypoints, 2) > 3
    [R_CW, t_CW, inliers] = p3pRANSAC(keypoints, landmarks, K, ...
        obj.p3pRANSACIt, obj.p3pTolerance, obj.minInliers, obj.adaptive,...
        obj.verbose);
else
    R_CW = [];
    t_CW = [];
    inliers = zeros(1, size(keypoints, 2));
end

%Update state
if ~isempty(R_CW) && ~isempty(t_CW)
    kpold = kpold(:, inliers);
    keypointsAll = keypoints;
    keypoints = keypoints(:, inliers);
    keypointsLost = [keypointsLost, keypointsAll(:, ~inliers)];
    landmarks = landmarks(:, inliers);
    
    disp('landmarks kept by RANSAC')
    sum(inliers)

end

%% Add new keypoints and landmarks
if size(keypoints, 2) < 130 && ~isempty(pose) && ~isempty(R_CW) && ~isempty(t_CW) && ~isempty(keypointsToAdd)
    
    
    pose1Matrix = reshape(pose, [3,4]);
    poseMatrix = [R_CW, t_CW];
    
    %Track the new keypoints
    [kpoldToAdd, keypointsToAdd, ~ , keepToAdd] =...
    obj.klt.KltTracker(inputHandler, ii, keypointsToAdd);
    
    if size(keypointsToAdd, 2) > 4     %TODO make 4 a variable among parameters
    %Landmarks triangulation
        landmarksToAdd = triangulationForKlt(K, kpoldToAdd,...
            keypointsToAdd, pose1Matrix, poseMatrix, 4);

        keypoints = [keypoints, keypointsToAdd];
        kpold = [kpold, keypointsToAdd];
        landmarks = [landmarks, landmarksToAdd];
        
        disp('................ADDED.............');
        size(landmarksToAdd, 2)
        disp('................LANDMARKS.............');

        landmarksAddedBool = true;
    else
        landmarksAddedBool = false;
    end
else
    landmarksAddedBool = false;
end

%% Generate new keypoints far from the ones we already have
image = inputHandler.getImage(ii);
keypointsThreshold = 100;    %at least n pixels far     TODO: move this elsewhere
addKeypointsKlt;
size(keypointsToAdd)

%Status stuff
trackedLandmarks = landmarks;
trackedKeypoints = keypoints;

if ~isempty(R_CW) && ~isempty(t_CW)
    pose = [R_CW(:); t_CW(:)];
    lostKeypoints = keypointsLost;
    landmarksIdx = landmarksIdx(keep);
    landmarksIdx = landmarksIdx(inliers);
     if landmarksAddedBool
        [obj.state, landmarksToAdd1, landmarksIdxToAdd, mask] ...
            = obj.state.addLandmarks(landmarksToAdd);
        landmarksIdx = [landmarksIdx; landmarksIdxToAdd];
     end
else
    pose = [];
end