function [trKp,kpMask] = KLT(image1,image2,kp1,r_T,nIt,lambda)
%KLT Summary of this function goes here
%   Detailed explanation goes here

    % Sampling
    image1_ = imresize(image1, 0.25); kp1 = kp1 / 4;
    image2_ = imresize(image2, 0.25);

    dkp = zeros(size(kp1));
    kpMask = true(1, size(kp1, 2));
    parfor j = 1:size(kp1, 2)
        [dkp(:,j), kpMask(j)] = trackKLTRobustly(image1_, image2_, ...
            kp1(:,j).', r_T, nIt, lambda);
    end
    kpMask = kpMask > 0;
    trKp = kp1(:,kpMask) + dkp(:,kpMask);

    % Interpolation
    trKp = trKp * 4;
    
    % Filter invalid keypoints
    valid = trKp(1,:) >= 1 & trKp(2,:) >= 1 ...
        & trKp(1,:) < size(image2,2) & trKp(2,:) < size(image2,2);
    trKp = trKp(:,valid);
    kpMask(kpMask > 0) = valid;
end

