verboseDisp(obj.verbose, ...
    '\n\nInit from frame %d\n=====================\n', prevFrameIdx);

[R_CW, t_CW] = obj.state.getLastPose();
prevPose = [R_CW, t_CW];

[trackedKeypoints, trackedLandmarks, pose, frameIdx, trackedCandidates, prevFrameKeypoints] ...
    = obj.initBlock.run(prevFrameIdx, prevPose);

lostKeypoints = [];
if ~isempty(pose)
    pose = reshape(pose(1:3,:), [], 1);
    trackedLandmarks = trackedLandmarks(1:3,:);
    
    % Add landmarks
    [trackedLandmarks, landmarksIdx, mask] = obj.state.addLandmarks(trackedLandmarks);
    
    % Add observations to prev frame
    obj.state.addLandmarksToPose(prevFrameIdx, ...
        landmarksIdx, prevFrameKeypoints(:,mask).');
    
    trackedKeypoints = trackedKeypoints(:, mask);
    
    if ~isempty(trackedCandidates) && obj.continuouslyTriangulate
        % Reset candidates (after init)
        obj.state.resetCandidates();
        obj.state.addCandidates(frameIdx, trackedCandidates);
    end
else
    prevFrameIdx = obj.state.goToPrevPose();
    assert(~isempty(prevFrameIdx), 'Hard reset needed.');
    nFrameProcessed = nFrameProcessed - 1;
end