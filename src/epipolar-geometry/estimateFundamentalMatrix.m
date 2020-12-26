function F = estimateFundamentalMatrix(p1,p2,normalizePoints)
% ESTIMATEFUNDAMENTALMATRIX Estimates the fundamental matrix from the image
% points correspondences p1 and p2 through the 8-points algorithm.
%
% F = ESTIMATEFUNDAMENTALMATRIX(p1,p2) returns the fundamental matrix
% estimated with the 8-points algorithm. The solution is in a least square
% sense and thus, more then 8 points can be provided.
% p1 is 3xN, the homogeneous image points in the first view
% p2 is 3xN, the homogeneous image points in the second view
% F is 3x3, the estimated fundamental matrix
% 
% ESTIMATEFUNDAMENTALMATRIX(p1,p2,normalizePoints) additionally allows to
% specify whether to normalize the points before to estimate the
% fundamental matrix; normalizePoints is thus a logical value.

arguments 
    p1 (3,:)
    p2 {mustBeEqualSize(p1,p2), atLeast8(p2)}
    normalizePoints logical = false
end

if normalizePoints
    [p1,T1] = points2DNormalization(p1);
    [p2,T2] = points2DNormalization(p2);
end

p1 = p1.';
p2 = p2.';

Q = [p1(:,1) .* p2(:,1), ...
    p1(:,2) .* p2(:,1), ...
    p1(:,3) .* p2(:,1), ...
    p1(:,1) .* p2(:,2), ...
    p1(:,2) .* p2(:,2), ...
    p1(:,3) .* p2(:,2), ...
    p1(:,1) .* p2(:,3), ...
    p1(:,2) .* p2(:,3), ...
    p1(:,3) .* p2(:,3)];

[~, ~, V] = svd(Q,0);
F = reshape(V(:, end), 3, 3).';

% Enforcing constraints on the decomposition of F
[U,~,V] = svd(F);
% S(3,3) = 0;
F = U * diag([1,1,0]) * V.';

if normalizePoints
    % Undo the normalization
    F = (T2.') * F * T1;
end

end

function mustBeEqualSize(a,b)
    if ~isequal(size(a),size(b))
        eid = 'Size:notEqual';
        msg = 'Size of first input must equal size of second input.';
        throwAsCaller(MException(eid,msg))
    end
end

function atLeast8(a)
    if size(a,2) < 8
        eid = 'Size:notEnoughPoints';
        msg = 'At least 8 points correspondences are required.';
        throwAsCaller(MException(eid,msg))
    end
end

