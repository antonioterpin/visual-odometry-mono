verboseDisp(obj.verbose, ...
    '\n\nInit from frame %d\n=====================\n', prevFrameIdx);

[R_CW, t_CW] = obj.state.getLastPose();
prevPose = [R_CW, t_CW];

[trackedKeypoints, trackedLandmarks, pose, frameIdx, prevFrameKeypoints,trackedCandidates,tracker] ...
    = obj.initBlock.run(prevFrameIdx, prevPose);

if ~isempty(pose)
    R_CW = pose(1:3,1:3);
    t_CW = pose(1:3,4);
    localized = true;
    
    trackedLandmarks = trackedLandmarks(1:3,:);
    
    % Add landmarks
    [trackedLandmarks, landmarksIdx, mask] = obj.state.addLandmarks(trackedLandmarks);
    
    % Add observations to prev frame
    obj.state.addLandmarksToPose(prevFrameIdx, ...
        landmarksIdx, prevFrameKeypoints(:,mask).');
    % Add observations to current frame
    obj.state.addPose(frameIdx, R_CW, t_CW);
    obj.state.addLandmarksToPose(frameIdx, landmarksIdx, trackedKeypoints.');
    
    trackedKeypoints = trackedKeypoints(:, mask);
    
    % BA immediately after init
    % Get optimization data structure
    [hiddenState, observations, bundleIdx] ...
        = obj.state.getOptimizationDS(obj.optBlock.maxBundleSize);

    % Start optimization
%     hiddenState = obj.optBlock.optimize(hiddenState, observations);

    % Update state
%     obj.state.optimizedBundle(hiddenState, bundleIdx);
    
    if ~isempty(trackedCandidates) && obj.continuouslyTriangulate
        % Reset candidates (after init)
        obj.state.resetCandidates();
        obj.state.addCandidates(frameIdx, trackedCandidates);
    end
else
    localized = false;
    prevFrameIdx = obj.state.gotoPrevPose();
    assert(~isempty(prevFrameIdx), 'Hard reset needed.');
    nProcessedFrames = nProcessedFrames - 2;
end