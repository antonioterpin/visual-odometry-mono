function plotFullTrajectory (poses)
    %PLOTFULLTRAJECTORY
    %Get extreme values
    minPosesX = min(poses(1, :));
    maxPosesX = max(poses(1, :));
    minPosesY = min(poses(3, :));
    maxPosesY = max(poses(3, :));
    plot(smooth(poses(1, :), 10), smooth(poses(3, :), 10),...   %changed to (1, :) instead of(:, 1) 12/12, 20.06
        '-x','MarkerSize', 2)
    axis ([minPosesX - 5, maxPosesX + 5, minPosesY - 5, maxPosesY + 5])
    title('Full Trajectory')
end