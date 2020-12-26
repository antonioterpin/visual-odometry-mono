function plotFullTrajectory (poses)
    %PLOTFULLTRAJECTORY
    %Get extreme values
    minPosesX = min(poses(1, :));
    maxPosesX = max(poses(1, :));
    minPosesY = min(poses(3, :));
    maxPosesY = max(poses(3, :));
    plot(poses(1, :), poses(3, :), '-x','MarkerSize', 2) % smooth eventually later
    axis ([minPosesX - 5, maxPosesX + 5, minPosesY - 5, maxPosesY + 5])
    title('Full Trajectory')
end