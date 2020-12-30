function patch = getWarpedPatch(I, W, x_T, r_T)

width = size(I,2); 
height = size(I,1);

[Idx, Jdx] = meshgrid(-r_T:r_T, -r_T:r_T);
numPix = numel(Idx);
w_idx = repmat(x_T.', 1, numPix) + W * [Idx(:).'; Jdx(:).'; ones(1, numPix)];
w_idx_ = floor(w_idx);

idx = r_T + [Idx(:).'; Jdx(:).'] + 1;

% keep only values warped within image boundaries
valid = all(w_idx_ > 1 & w_idx_ < [width; height], 1);
w_idx = w_idx(:, valid);
w_idx_ = w_idx_(:, valid);
idx = idx(:, valid);

patchSize = 2*r_T.' + 1;
patch = zeros(patchSize);

% this is an approx
% I = double(padarray(I, [1 1], 'replicate')); % easier bilinear interpolation
% % Build warped patch
ab = (w_idx - w_idx_).'; % bilinear interpolation coefficients

% % For convenience, we use linear indexing
I = double(I);
idx = sub2ind(size(patch), idx(2,:).', idx(1,:).');
shift = ones(size(w_idx_,2), 1);
w_idx_00 = sub2ind(size(I), w_idx_(2,:).', w_idx_(1,:).');
w_idx_01 = w_idx_00 + size(I,1)*shift; % sub2ind(size(I), w_idx_(2,:).'+1, w_idx_(1,:).');
w_idx_10 = w_idx_00 + shift; % sub2ind(size(I), w_idx_(2,:).', w_idx_(1,:).'+1);
w_idx_11 = w_idx_01 + shift; % sub2ind(size(I), w_idx_(2,:).'+1, w_idx_(1,:).'+1);

patch(idx) = (1-ab(:,2)) .* ((1-ab(:,1)) .* I(w_idx_00) + ab(:,1) .* I(w_idx_01)) ...
    + ab(:,2) .* ((1-ab(:,1)) .* I(w_idx_10) + ab(:,1) .* I(w_idx_11));
end