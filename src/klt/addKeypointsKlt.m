[keypointsToAdd, ~] = obj.coBlock.Detector.extractFeatures(image);
keypointsToAdd = keypointsToAdd(:, 1:250);  %take only best N
idxKeypointsToRemove = [];
tic
for i = 1 : size(keypoints, 2)
    if any(intersect(any(any(keypointsToAdd(1, :) < keypoints(1, i)+keypointsThreshold)),...
            any(any(keypointsToAdd(1, :) > keypoints(1, i)-keypointsThreshold))))
        idxX = find(keypointsToAdd(1, :) < keypoints(1, i)+keypointsThreshold);
        idxY = find(keypointsToAdd(1, :) > keypoints(1, i)-keypointsThreshold);
        idxKeypoints = intersect(idxX, idxY);
        if ~isempty(idxKeypoints)
            for j = 1 : length(idxKeypoints)
                if any(intersect(any(keypointsToAdd(2, j) < keypoints(2, i)+keypointsThreshold),...
                        any(keypointsToAdd(2, j) > keypoints(2, i)-keypointsThreshold)))
                    idxKeypointsToRemove = [idxKeypointsToRemove, idxKeypoints(j)];
                end
            end
        end
    end
end
toc
keypointsToAdd(:, idxKeypointsToRemove) = [];