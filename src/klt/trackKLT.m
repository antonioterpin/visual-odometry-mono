function W = trackKLT(I_R, I, x_T, r_T, num_iters, W)
if nargin < 6
   W = getSimWarp(0, 0, 0, 1); % identity
end

% T suffix indicates image evaluated for patch T
t = getWarpedPatch(I_R, W, x_T, r_T); % patch of reference image

% x and y coordinates of the patch also never change
xs = -r_T:r_T;
ys = -r_T:r_T;
n = numel(xs);
X = ones(n,1)*xs;
Y = ys.'*ones(1,n);
dwdp1 = [X, zeros(n), Y, zeros(n), ones(n), zeros(n)];
dwdp2 = [zeros(n), X, zeros(n), Y, zeros(n), ones(n)];

for iter = 1:num_iters
    wp = getWarpedPatch(I, W, x_T, r_T + 1);
    i = wp(2:end-1, 2:end-1);
    
    didw1 = conv2(1, [1 0 -1], wp(2:end-1, :), 'valid');
    didw2 = conv2([1 0 -1], 1, wp(:, 2:end-1), 'valid');
    
    didp = repmat(didw1, 1, 6) .* dwdp1 + repmat(didw2, 1, 6) .* dwdp2; % nx6n
    
    % reshaping in a convenient manner
    e = reshape(t - i, n*n, 1); % nxn
    didp = reshape(didp, n*n, 6);
    
    delta_p = pinv(didp) * e;
    W = W + reshape(delta_p, 2, 3);
    
    if norm(delta_p) < 1e-3
       return
    end
end

end