function [delta_keypoint, keep] = trackKLTRobustly(I_prev, I, keypoint, ...
    r_T, num_iters, lambda, W0)
% I_prev: reference image, I: image to track point in, keypoint: point to 
% track, expressed as [x y]=[col row], r_T: radius of patch to track, 
% num_iters: amount of iterations to run, lambda: bidirectional error
% threshold; delta_keypoint: delta by which the keypoint has moved between 
% images, (2x1), keep: true if the point tracking has passed the
% bidirectional error test.

W0inv = [W0(:,1:2).', -W0(:,1:2).'*W0(:,end)];

W = trackKLT(I_prev, I, keypoint, r_T, num_iters, W0);
delta_keypoint = W(:, end);
Winv = trackKLT(I, I_prev, (keypoint'+delta_keypoint)', r_T, num_iters, W0inv);
dkpinv = Winv(:, end);
keep = norm(delta_keypoint + dkpinv) < lambda;

end

