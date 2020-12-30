verboseDisp(obj.verbose, ...
    '\n\nProcessing frame %d\n=====================\n', frameIdx);

% Get previous observations
[landmarks, landmarksIdx, kp] ...
    = obj.state.getObservations(prevFrameIdx);

candidates = [];
if obj.continuouslyTriangulate
    candidates = obj.state.getCandidates();
end

% TODO: is keyframe selection + plotting candidates possible without changing everything?
[R_CW, t_CW, trackedKeypoints, kpMask, trackedCandidates, trackedCandidatesMask, newKpc] ...
    = obj.coBlock.localize(prevFrameIdx, frameIdx, kp, landmarks, candidates);

if isempty(R_CW) || isempty(t_CW)
    localized = false;
else
    localized = true;
    % Update state
    landmarksIdx = landmarksIdx(kpMask);
    trackedLandmarks = landmarks(:, kpMask);
    obj.state.addPose(frameIdx, R_CW, t_CW);
    obj.state.addLandmarksToPose(frameIdx, landmarksIdx, trackedKeypoints.');
    
    % Triangulation
    if obj.continuouslyTriangulate
        obj.state.evaluateCandidates(K, trackedCandidatesMask, trackedCandidates);
        obj.state.addCandidates(frameIdx, newKpc);
    end
    
    % display also new candidates
    trackedCandidates = [trackedCandidates, newKpc];
end