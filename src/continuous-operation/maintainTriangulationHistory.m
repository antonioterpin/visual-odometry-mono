function [sizeHistory, lastKeypoints, lastDescriptors] =...
    maintainTriangulationHistory (sizeHistory, lastKeypoints, lastDescriptors,...
    keypoints, descriptors)
    %MAINTAINTRIANGULATIONHISTORY

% Accumulate data for triangulation
lastKeypoints = [lastKeypoints, keypoints];
lastDescriptors = [lastDescriptors, descriptors];
if length(sizeHistory) >= 2
    % Reduce historical data to last 3 steps
    if size(lastKeypoints, 2) >= sum(sizeHistory(end-1 : end))
        lastKeypoints = lastKeypoints(:, (end - sum(sizeHistory(end-1 : end)) -1) : end);
        lastDescriptors = lastDescriptors(:, (end - sum(sizeHistory(end-1 : end)) -1) : end);
    end        
end
sizeHistory = [sizeHistory, size(keypoints, 2)];
end