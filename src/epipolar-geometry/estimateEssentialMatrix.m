function E = estimateEssentialMatrix(p1, p2, K1, K2)
% ESTIMATEESSENTIALMATRIX Estimate essential matrix from observations.
%
% E = ESTIMATEESSENTIALMATRIX(p1,p2) is the estimated essential matrix
% given the calibrated coordinates p1 and p2.
% p1 and p2 are 3xM matrices, where M is the number of observations [u; v; 1].
% 
% E = ESTIMATEESSENTIALMATRIX(p1,p2,K) accepts uncalibrated image
% coordinates p1 and p2, but needs the intrinsic matrix K, which is the
% same for both the cameras.
% K is a 3x3 intrinsics matrix.
%
% E = ESTIMATEESSENTIALMATRIX(p1,p2,K1,K2) allows the two views to have
% different camera intrinsics.

arguments
    p1 (3,:)
    p2 {mustBeEqualSize(p1,p2)}
    K1 (3,3) = []
    K2 (3,3) {bothEmptyOrBothValid(K1,K2)} = K1
end

if ~isempty(K1) && ~isempty(K2)
    p1 = K1 \ p1;
    p2 = K2 \ p2;
end

% Notice that we pre-divided the image points by K1 and K2.
E = estimateFundamentalMatrix(p1, p2);
end

function mustBeEqualSize(a,b)
    if ~isequal(size(a),size(b))
        eid = 'Size:notEqual';
        msg = 'Size of first input must equal size of second input.';
        throwAsCaller(MException(eid,msg))
    end
end

function bothEmptyOrBothValid(K1,K2)
    if (isempty(K1) && ~isempty(K2)) || (isempty(K2) && ~isempty(K1))
        eid = 'Size:notValid';
        msg = 'Both K1 and K2 must be 3x3.';
        throwAsCaller(MException(eid,msg))
    end
end