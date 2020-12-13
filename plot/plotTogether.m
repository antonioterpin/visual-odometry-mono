function plotTogether (image, inliers, outliers, poses, landmarks,...
    landmarksHistory, nSkip, nStart)
    %PLOTTOGETHER
    % image is the current image
    % inliers and outliers: [Nx2] [U,V] matrices
    % poses is the [Nx3] vector of translation of the camera frame,
    %    where each entry is computed as (-R_IC.' * t_IC)'.
    % landmarks is the [Nx3] matrix of the current set of landmarks
    
    % TODO Set axis in the correct way
    
    figure(1)
    % Image with keypoints
    subplot(2,2,1)
    plotCurrentImage(image, inliers, outliers)
    
    % Last N poses and current landmarks
    if(size(poses, 2) > 20)
        posesLast = poses(:, end-19 : end);
    else
        posesLast = poses;
    end
    subplot(1,2,2)
    plotRecentTrajectory(posesLast, landmarks)
    
    % Number of triangulated landmarks
    subplot(2,4,5)
    plotLandmarksHistory(landmarksHistory, nSkip, nStart)
    
    % Full trajectory
    subplot(2,4,6)
    plotFullTrajectory(poses)
end