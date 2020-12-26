verboseDisp(obj.verbose, ...
    '\n\nInit from frame %d\n=====================\n', ii - obj.nSkip);

[R_CW, t_CW] = obj.state.getLastPose();
prevPose = [R_CW, t_CW];

[trackedKeypoints, trackedLandmarks, ~, ...
    pose, ii, unmatchedKeypoints, unmatchedDescriptors] = ...
    obj.initBlock.run(inputHandler, K, ii-obj.nSkip, prevPose);

lostKeypoints = [];
if ~isempty(pose)
    pose = reshape(pose(1:3,:), [], 1);
    trackedLandmarks = trackedLandmarks(1:3,:);

    % TODO add observation also to previous frame
    [obj.state, trackedLandmarks, landmarksIdx, mask] ...
        = obj.state.addLandmarks(trackedLandmarks);
    trackedKeypoints = trackedKeypoints(:, mask);
    
    if ~isempty(unmatchedKeypoints)
%         obj.state = obj.state.addCandidates(ii, ...
%             unmatchedKeypoints, unmatchedDescriptors);
    end
else
    poseHistory = poseHistory(:, 1:end - 1);
    landmarksHistory = landmarksHistory(1:end-1);
end