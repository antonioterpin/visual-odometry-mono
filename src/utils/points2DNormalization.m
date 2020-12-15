function [normalizedPoints, T] = points2DNormalization(points)
% POINTS2DNORMALIZATION Normalizes 2D homogeneous points in a convenient
% manner.
%
% [normalizedPoints, T] = points2DNormalization(points) returns the
% normalized points and the linear transformation used to normalize them.
% points is 3xN, the homogeneous coordinates to normalize.
% normalizedPoints is 3xN, the homogeneous normalized coordinates.
% T is 3x3, the normalization trasformation matrix.

arguments
    points (3,:)
end

N = size(points,2);

points = points ./ repmat( points(3,:),3,1);

mu = mean(points(1:2,:),2);

centroids = points(1:2,:) - repmat(mu, 1, N);
sigma = sqrt(mean(sum(centroids.^2)));

s = sqrt(2) / sigma;
T = [s 0 -s*mu(1);
    0 s -s*mu(2);
    0 0 1];
 
normalizedPoints = T * points;

end     