function plotCurrentImage (image, inliers, outliers)
    %PLOTCURRENTIMAGE Plots the upper left part of the output.
    
    %inliers and outliers: [Nx2] [U,V] matrices
    
    imshow(image);
    hold on;
    if(~isempty(inliers))
        scatter(inliers(:, 1), inliers(:, 2), 15, 'g', 'filled');
    end
    if(~isempty(outliers))
<<<<<<< HEAD:src/plot/plotCurrentImage.m
        scatter(outliers(1, :), outliers(2, :), 15, 'r', 'filled');
=======
        scatter(outliers(2, :), outliers(1, :), 15, 'r', 'filled');
>>>>>>> dbabe5a106c153f1ef9ed7e5182053ddc2780624:plot/plotCurrentImage.m
    end
    hold off;
    title('Current Image');
end