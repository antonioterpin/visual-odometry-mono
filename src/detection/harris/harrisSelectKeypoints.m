function keypoints = harrisSelectKeypoints(scores, N, r)
% HARRISSELECTKEYPOINTS Selects the best N keypoints given the
% harris scores. A non maxima suppression of radius r is performed.
%
% keypoints = HARRISSELECTKEYPOINTS(scores, N, r) returns the selected keypoints 
% from the harris scores.
% keypoints is 2xN; [u; v].
% r is the scalar radius of the non maxima suppression
% 
% Example:
% scores = harris(image, patchSize, kappa);
% keypoints = harrisSelectKeypoints(scores, N, r);
%
% See also harrisScore, harrisDescribeKeypoints

% % Apparently slower
% % scores_suppressed = col2im(...
% %     nonMaximaSuppression(im2col(scores, [2*r+1,2*r+1], 'sliding')), ...
% %     [2*r+1,2*r+1], size(scores));
% scores_suppressed = colfilt(scores, [2*r+1,2*r+1], 'sliding', @nonMaximaSuppression);
% [~,kp] = maxk(scores_suppressed(:), keypointsNumber);
% [v, u] = ind2sub(size(scores_suppressed), kp);
% keypoints = [u.'; v.'];

keypoints = zeros(2, N);
temp_scores = padarray(scores, [r r]);
for i = 1:N
    [~, kp] = max(temp_scores(:));
    [row, col] = ind2sub(size(temp_scores), kp);
    kp = [col;row];
    keypoints(:, i) = kp - r;
    temp_scores(kp(2)-r:kp(2)+r, kp(1)-r:kp(1)+r) = ...
        zeros(2*r + 1, 2*r + 1);
end

end

% function out = nonMaximaSuppression(col)
%     [m, i] = max(col);
%     out = m .* (i == repmat((1+size(col,1))/2, 1, size(col,2)));
% end

