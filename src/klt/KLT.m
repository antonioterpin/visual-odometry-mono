function [trKp,kpMask] = KLT(image1,image2,kp1,r_T,nIt,lambda,nScales)
%KLT Summary of this function goes here
%   Detailed explanation goes here

    if nargin < 7
        nScales = 1;
    end
    
    baseDownsampling = 0.25;
    
    W0 = reshape(getSimWarp(0, 0, 0, 1), [], 1);
    W0 = repmat(W0, 1, size(kp1, 2));

    for it = nScales : -1 : 2
        scale = baseDownsampling^it;

        parfor j = 1:size(kp1, 2)
            W0_j = reshape(W0(:, j), [2,3]);
            W0_j = trackKLT(...
                imresize(image1, scale), imresize(image2, scale), ...
                kp1(:,j).' * scale, ceil(r_T * scale), nIt, W0_j);
            W0_j(:, end) = W0_j(:, end) / baseDownsampling;
            W0(:, j) = reshape(W0_j, [], 1);
        end
    end
    
    dkp = zeros(size(kp1));
    kpMask = true(1, size(kp1, 2));
    scale = baseDownsampling;
    parfor j = 1:size(kp1, 2)
        W0_j = reshape(W0(:, j), [2,3]);
        [dkp(:,j), kpMask(j)] = trackKLTRobustly(...
            imresize(image1, scale), imresize(image2, scale), ...
            kp1(:,j).' * scale, ceil(r_T * scale), nIt, lambda, W0_j);
    end
    trKp = kp1(:,kpMask) + dkp(:,kpMask) / scale;
    % Interpolation
    
    valid = trKp(1,:) >= 1 & trKp(2,:) >= 1 ...
        & trKp(1,:) < size(image2,2) & trKp(2,:) < size(image2,1);  %TODO Correct this also in develop  size(image2,1)
    trKp = trKp(:,valid);
    kpMask(kpMask > 0) = valid;
    
    kp1 = kp1(:, kpMask);
    figure(8)
    imshow(image2);
    hold on;
    plotMatches(1:size(trKp, 2), flipud(trKp), flipud(kp1));
    hold off;
    pause(0.05);
end

