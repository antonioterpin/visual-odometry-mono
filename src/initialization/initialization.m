verboseDisp(obj.verbose, ...
    '\n\nInit from frame %d\n=====================\n', prevFrameIdx);

[R_CW, t_CW] = obj.state.getLastPose();
prevPose = [R_CW, t_CW];

[trackedKeypoints, trackedLandmarks, ~, ...
    pose, frameIdx, unmatchedKeypoints, unmatchedDescriptors, ...
    prevFrameKeypoints] = ...
    obj.initBlock.run(inputHandler, K, prevFrameIdx, prevPose);

lostKeypoints = [];
if ~isempty(pose)
    pose = reshape(pose(1:3,:), [], 1);
    trackedLandmarks = trackedLandmarks(1:3,:);
    
    % Add landmarks
    [trackedLandmarks, landmarksIdx, mask] = obj.state.addLandmarks(trackedLandmarks);
    
    % Add observations to prev frame
    obj.state.addLandmarksToPose(prevFrameIdx, landmarksIdx, prevFrameKeypoints.');
    
    trackedKeypoints = trackedKeypoints(:, mask);
    
    if ~isempty(unmatchedKeypoints)
%         obj.state = obj.state.addCandidates(ii, ...
%             unmatchedKeypoints, unmatchedDescriptors);
    end
else
    prevFrameIdx = obj.state.goToPrevPose();
    assert(~isempty(prevFrameIdx), 'Hard reset needed.');
    nFrameProcessed = nFrameProcessed - 1;
end