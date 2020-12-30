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
[R_CW, t_CW, trackedKeypoints, kpMask, trackedCandidates, kpcMask, newKpc] ...
    = obj.coBlock.localize(prevFrameIdx, frameIdx, kp, landmarks, candidates);

if isempty(R_CW) || isempty(t_CW)
    pose = [];
else
    pose = [R_CW(:); t_CW(:)];
    landmarksIdx = landmarksIdx(kpMask);
    trackedLandmarks = landmarks(:, kpMask);
    if obj.continuouslyTriangulate
        obj.state.evaluateCandidates(K, kpcMask, trackedCandidates);
        obj.state.addCandidates(frameIdx, newKpc);
    end
end