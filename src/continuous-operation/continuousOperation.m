verboseDisp(obj.verbose, ...
    '\n\nProcessing frame %d\n=====================\n', frameIdx);

% Get previous observations
[landmarks, landmarksIdx, kp] ...
    = obj.state.getObservations(prevFrameIdx);

candidates = [];
if obj.continuouslyTriangulate
    candidates = obj.state.getCandidates();
end

for it = frameIdx:frameIdx+obj.coBlock.keyframeMaxSkip
    [R_CW, t_CW, trackedKeypoints, kpMask, trackedCandidates, trackedCandidatesMask] ...
        = obj.coBlock.localize(prevFrameIdx, frameIdx, kp, landmarks, candidates, tracker);
    
    if nnz(kpMask) / numel(kpMask) < obj.coBlock.keyframeConfidence
        break;
    end
end

frameIdx = it;
verboseDisp(obj.verbose, ...
        'Next keyframe is %d.\n', frameIdx);

if isempty(R_CW) || isempty(t_CW)
    localized = false;
else
    localized = true;
    
    % Candidate new keypoints
    error = max(0, obj.coBlock.nLandmarksReference - size(trackedKeypoints, 2) - size(trackedCandidates,2));
    newKpc = [];
    if error > 0
        image2 = obj.inputBlock.getImage(frameIdx);
        mask = obj.coBlock.detector.getMask(...
            size(image2), floor([trackedKeypoints, trackedCandidates]), ...
            obj.coBlock.candidateSuppressionRadius);

        newCandidates = repmat(...
            floor(error / prod(obj.coBlock.samplingSize)),...
            reshape(obj.coBlock.samplingSize, 1, 2));
        newKpc = obj.coBlock.detector.extractFeatures(...
            image2, newCandidates, mask);
    end
    
    % Update state
    landmarksIdx = landmarksIdx(kpMask);
    trackedLandmarks = landmarks(:, kpMask);
    obj.state.addPose(frameIdx, R_CW, t_CW);
    obj.state.addLandmarksToPose(frameIdx, landmarksIdx, trackedKeypoints.');
    
    % Triangulation
    if obj.continuouslyTriangulate
        [confirmedKp, confirmedLandmarks, trackedCandidates] ...
            = obj.state.evaluateCandidates(K, trackedCandidatesMask, trackedCandidates);
        
        obj.state.addCandidates(frameIdx, newKpc);
        
        trackedKeypoints = [trackedKeypoints, confirmedKp];
        trackedLandmarks = [trackedLandmarks, confirmedLandmarks];
    end
    
    % display also new candidates
	trackedCandidates = [trackedCandidates, newKpc];
end