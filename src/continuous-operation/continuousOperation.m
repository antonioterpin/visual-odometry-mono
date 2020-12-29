verboseDisp(obj.verbose, ...
    '\n\nProcessing frame %d\n=====================\n', frameIdx);

% Get previous observations
[landmarks, landmarksIdx, kp] ...
    = obj.state.getObservations(prevFrameIdx);

candidates = [];
if obj.continuouslyTriangulate
    % TODO state should need only kpcidx and matchedkpcidx
end

% TODO: is keyframe selection + plotting candidates possible without changing everything?
[R_CW, t_CW, trKp, kpMask, trKpc, kpcMask, newKpc] = ...
    obj.coBlock.localize(prevFrameIdx, frameIdx, kp, landmarks, candidates);

% TODO wtf
% trackedKeypoints = kp(:,trKpMask);
% trackedLandmarks = landmarks(:,trKpMask);
% TODO handle candidates evaluation

if isempty(R_CW) || isempty(t_CW)
    pose = [];
else
    pose = [R_CW(:); t_CW(:)];
    landmarksIdx = landmarksIdx(kpMask);
    kpold = kp(:, kpMask);
    keypoints = trKp;
    landmarks = landmarks(:, kpMask);
end

%% Add new keypoints and landmarks
if size(keypoints, 2) < 130 && ~isempty(pose) && ~isempty(R_CW) && ~isempty(t_CW) && ~isempty(keypointsToAdd)
    
    pose1Matrix = reshape(pose, [3,4]);
    poseMatrix = [R_CW, t_CW];
    
    %Track the new keypoints
    [trKp, kpMaskToAdd, ~] = obj.coBlock.track(prevFrameIdx, frameIdx, keypointsToAdd);
    kpoldToAdd = keypointsToAdd(:,kpMaskToAdd);
    keypointsToAdd = trKp;
    
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
keypointsToAdd = newKpc;

%Status stuff
trackedLandmarks = landmarks;
trackedKeypoints = keypoints;

if ~isempty(R_CW) && ~isempty(t_CW)
     if landmarksAddedBool
        [landmarksToAdd1, landmarksIdxToAdd, mask] ...
            = obj.state.addLandmarks(landmarksToAdd);
        landmarksIdx = [landmarksIdx; landmarksIdxToAdd];
     end
end
