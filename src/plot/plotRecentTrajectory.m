function plotRecentTrajectory (poses, landmarks)
    %PLOTRECENTTRAJECTORY
    
    % poses is the [Nx3] vector of translation of the camera frame,
    % where each entry is computed as (-R_IC.' * t_IC)'.
    % Thus the three components are x, y and z.
    
    % landmarks is the [Nx3] matrix of the current set of landmarks
    
    %Get extreme values
    minPosesX = min(poses(1, :));
    maxPosesX = max(poses(1, :));
    minPosesY = min(poses(3, :));
    maxPosesY = max(poses(3, :)); 
    
    plot(poses(1, :), poses(3, :), '-x','MarkerSize', 2) % smooth later eventually
    hold on;
    scatter(landmarks(1, :), landmarks(3, :), 4, 'k')
%     set(gcf, 'GraphicsSmoothing', 'on');
    view(0,90);
    hold off;
    axis([minPosesX-15, maxPosesX+15, minPosesY-10, maxPosesY+20])
    title('Recent Trajectory and Current Landmarks')
end