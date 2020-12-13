function plotLandmarksHistory (landmarksHistory, nSkip, nStart)
    %PLOTLANDMARKSHISTORY Plots the number of the landmarks over the past
    %iterations
    
    %nSkip is the number of skipped frames in the main iteration.
    % For instance, for 1 : 2 : lastImageIndex, nSkip = 2.
    if length(landmarksHistory) < 20
        plot([1:length(landmarksHistory)] .* nSkip + nStart, landmarksHistory, '-k')
    else
        plot([length(landmarksHistory)-19 : length(landmarksHistory)] .* nSkip + nStart,...
            landmarksHistory(end-19 : end), '-k')
    end
    title('Landmarks History')
end