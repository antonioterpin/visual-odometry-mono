function descriptors = harrisDescribeKeypoints(image, keypoints, r)
% Returns a MxN matrix of image patch vectors based on image
% img and a 2xN matrix containing the keypoint coordinates.
% r is the patch "radius".
%
% Example:
% scores = harrisScore(image, patchSize, kappa);
% keypoints = harrisSelectKeypoints(scores, N, r);
% descriptors = harrisDescribeKeypoints(image, keypoints, descriptorRadius);
%
% See also harrisScore, harrisSelectKeypoints

arguments
    image
    keypoints 
    r {mustBePositive(r)}
end

descriptorSize = 2*r+1;
M = descriptorSize^2;
N = size(keypoints,2);

descriptors = uint8(zeros(M, size(keypoints,2)));
padded = padarray(image, [r, r]);

for i = 1 : N
    kp = keypoints(:, i) + r;
    descriptors(:,i) = reshape(...
        padded(kp(2)-r:kp(2)+r, kp(1)-r:kp(1)+r), [], 1);
end

% 
% range = repmat(0:descriptorSize-1, N, 1);
% I = repmat(keypoints(1,:).', 1, descriptorSize) + range;
% J = repmat(keypoints(2,:).', 1, descriptorSize) + range;
%
% % Something like meshgrid is needed here...
% 
% I = reshape(I.', [], 1);
% J = reshape(J.', [], 1);
% 
% imageIndices = sub2ind(size(padded), J, I);
% descriptors = reshape(padded(imageIndices), M, []);
end
