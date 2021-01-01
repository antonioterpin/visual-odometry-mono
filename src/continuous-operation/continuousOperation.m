verboseDisp(obj.verbose, ...
    '\n\nProcessing frame %d\n=====================\n', frameIdx);

% Get previous observations
[landmarks, landmarksIdx, kp] ...
    = obj.state.getObservations(prevFrameIdx);

trackedCandidates = [];
if obj.continuouslyTriangulate
    trackedCandidates = obj.state.getCandidates();
end

trackedKeypoints = kp;
kpMask = true(size(kp,2),1);
trackedCandidatesMask = true(size(trackedCandidates,2), 1);
trackedLandmarks = landmarks;
N = numel(kpMask);
for it = frameIdx:frameIdx+obj.coBlock.keyframeMaxSkip
    [R_CW, t_CW, trackedKeypoints, kpMask, trackedCandidates, trackedCandidatesMask_] ...
        = obj.coBlock.localize(prevFrameIdx, it, ...
        trackedKeypoints, trackedLandmarks, trackedCandidates, tracker);
    
    landmarksIdx = landmarksIdx(kpMask);
    trackedLandmarks = trackedLandmarks(:, kpMask);
    trackedCandidatesMask(trackedCandidatesMask) = trackedCandidatesMask_;
    
    if nnz(kpMask) / N < obj.coBlock.keyframeConfidence
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
    
    % Update state
    obj.state.addPose(frameIdx, R_CW, t_CW);
    obj.state.addLandmarksToPose(frameIdx, landmarksIdx, trackedKeypoints.');
    
    % Triangulation
    if obj.continuouslyTriangulate
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
        
        [confirmedKp, confirmedLandmarks, trackedCandidates] ...
            = obj.state.evaluateCandidates(K, trackedCandidatesMask, trackedCandidates);
        
        obj.state.addCandidates(frameIdx, newKpc);
        
        trackedKeypoints = [trackedKeypoints, confirmedKp];
        trackedLandmarks = [trackedLandmarks, confirmedLandmarks];
        % display also new candidates
        trackedCandidates = [trackedCandidates, newKpc];
    end
end