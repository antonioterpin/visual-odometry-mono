function scores = harrisScore(image, patchSize, kappa)
% HARRISSCORE Calculates the harris scores on the provided image.
% 
% scores = harris(image, patchSize, kappa) returns the harris scores on the
% provided image. patchSize and kappa are the parameters of the harris
% corner detector. size(scores) == size(image)
% 
% Example:
% scores = harrisScore(image, patchSize, kappa);
% keypoints = harrisSelectKeypoints(scores, N, r);
% descriptors = harrisDescribeKeypoints(image, keypoints, descriptorRadius);
%
% See also harrisSelectKeypoints, harrisDescribeKeypoints

arguments 
    image
    patchSize {mustBePositive(patchSize)}
    kappa {mustBePositive(kappa)}
end

[Ix, Iy] = imgradientxy(image, 'sobel');
Ixx = double(Ix .^ 2);
Iyy = double(Iy .^ 2);
Ixy = double(Ix .* Iy);

patch = ones(patchSize, patchSize) / (patchSize ^ 2);
sIxx = conv2(Ixx, patch, 'valid');
sIyy = conv2(Iyy, patch, 'valid');
sIxy = conv2(Ixy, patch, 'valid');

scores = (sIxx .* sIyy - sIxy .^ 2) ... determinant
    - kappa * (sIxx + sIyy) .^ 2;  % square trace

scores(scores < 0) = 0;

scores = padarray(scores, (size(image) - size(scores)) / 2);

end
