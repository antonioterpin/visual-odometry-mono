function model = relativePoseFromSample(sample, otherParams)
    K = reshape(otherParams(1:9), 3, 3);
    p1 = sample(1:3,:);
    p2 = sample(4:6,:);
    
    T_21 = estimateRelativePose(p1, p2, K, K);
    
    model = T_21;
end