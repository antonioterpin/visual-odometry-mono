function p_GC = alignEstimateToGroundTruth(pp_GC, p_VC)
% TODO DOCUMENTATION!

N = size(p_VC, 2);

% 1. Centroids
p = mean(p_VC, 2);
q = mean(pp_GC, 2);

% 2. Center points wrt centroids
x = p_VC - p;
y = pp_GC - q;

varx = sum(mean(x.^2, 2));

% 3. Covariance matrix
K = (x * y.') / N;

% 4. Rotation matrix
[U,D,V] = svd(K);
sign = 2 * (det(K) >= 0) - 1;
S = eye(size(V,1)); 
S(end,end) = sign;

R_GV = U*V.' * det(U*V.');

% 5. Scale
c = trace(D*S) * (varx + 10^-5).^-1;

% 6. Translation
t_GV = q - c * R_GV * p;

p_GC = c * R_GV * p_VC + t_GV;

end