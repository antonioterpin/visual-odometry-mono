verboseDisp(obj.verbose, ...
    '\n\nProcessing frame %d\n=====================\n', frameIdx);

% Get previous observations
[landmarks, landmarksIdx, kp] ...
    = obj.state.getObservations(prevFrameIdx);

candidates = [];
if obj.continuouslyTriangulate
    candidates = obj.state.getCandidates();
end

[R_CW, t_CW, trackedKeypoints, kpMask, ...
    trackedCandidates, trackedCandidatesMask, newKpc] ...
        = obj.coBlock.localize(prevFrameIdx, frameIdx, kp, landmarks, candidates, tracker);

% Keyframe selection
it = frameIdx + obj.nSkip;
while nnz(kpMask) / numel(kpMask) > obj.coBlock.keyframeConfidence ...
        && ~isempty(R_CW) && ~isempty(t_CW) ...
        && it - prevFrameIdx <= obj.coBlock.keyframeMaxSkip
    verboseDisp(obj.verbose, ...
        'Considering frame %d as keyframe.\n', frameIdx);
    [R_CW_, t_CW_, trackedKeypoints_, kpMask_, ...
        trackedCandidates_, trackedCandidatesMask_, newKpc_] ...
            = obj.coBlock.localize(frameIdx, frameIdx+obj.nSkip, ...
                trackedKeypoints, landmarks(:,kpMask), trackedCandidates, tracker);
            
    if nnz(kpMask_) / numel(kpMask_) > obj.coBlock.keyframeConfidence...
            && ~isempty(R_CW_) && ~isempty(t_CW_)
        R_CW = R_CW_;
        t_CW = t_CW_;
        trackedKeypoints = trackedKeypoints_;
        kpMask(kpMask > 0) = kpMask_;
        trackedCandidates = trackedCandidates_;
        trackedCandidatesMask(trackedCandidatesMask > 0) = trackedCandidatesMask_;
        newKpc = newKpc_;
        frameIdx = it;
    end
    it = it + obj.nSkip;
end

verboseDisp(obj.verbose, ...
        'Next keyframe is %d.\n', frameIdx);

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
        [confirmedKp, confirmedLandmarks, trackedCandidates] ...
            = obj.state.evaluateCandidates(K, trackedCandidatesMask, trackedCandidates);
        
%         if size(trackedKeypoints, 2) + size(trackedCandidates, 2) ...
%                 < 0.8 * obj.coBlock.nLandmarksReference
        obj.state.addCandidates(frameIdx, newKpc);
%         end
        
        trackedKeypoints = [trackedKeypoints, confirmedKp];
        trackedLandmarks = [trackedLandmarks, confirmedLandmarks];
    end
    
    % display also new candidates
	trackedCandidates = [trackedCandidates, newKpc];
end