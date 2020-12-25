function [isInlier, error] = errorMetricEpipolarLineDistance(p1, p2, otherParams)
% TODO update doc
% ERRORMETRIC Find inlier points in p1 and p2 with respect to an error
% metric (e.g., epipolar line distance).
%
% isInlier = errorMetric(p1, p2, otherParams) Return the logical indices
% marking which elements of p1 and p2 are inliers.
% p1 is 3xN, homogenous image coordinates in the first frame [u; v; 1].
% p2 is 3xN, homogenous image coordinates in the second frame [u; v; 1].
%
% [..., error] = errorMetric(...) Additionally returns the values of the
% errors corresponding to a pair of observation (p1(:,i), p2(:,i)).

% TODO PROPERLY DEFINE THE SIGNATURE OF THIS FUNCTION, PARAMS, ..

arguments
    p1 (3,:)
    p2 {mustBeEqualSize(p1,p2)}
    otherParams
end

% global triangulationTolerance;
assert(numel(otherParams) == 9, ...
    'otherParams must have 9 elements to be reshaped into a 3x3 fundamental matrix');
F = reshape(otherParams, 3, 3);

error = epipolarLineDistance(F,p1,p2);
isInlier = error < 1;

end

function mustBeEqualSize(a,b)
    if ~isequal(size(a),size(b))
        eid = 'Size:notEqual';
        msg = 'Size of first input must equal size of second input.';
        throwAsCaller(MException(eid,msg))
    end
end
