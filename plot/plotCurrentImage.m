function plotCurrentImage (image, inliers, outliers)
    %PLOTCURRENTIMAGE Plots the upper left part of the output.
    
    %inliers and outliers: [Nx2] [U,V] matrices
    
    imshow(image);
    hold on;
    if(~isempty(inliers))
        scatter(inliers(:, 1), inliers(:, 2), 15, 'g', 'filled');
    end
    if(~isempty(outliers))
        scatter(outliers(:, 1), outliers(:, 2), 15, 'r', 'filled');
    end
    hold off;
    title('Current Image');
end