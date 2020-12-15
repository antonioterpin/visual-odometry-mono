function d = epipolarLineDistance(F,p1,p2)
% EPIPOLARLINEDISTANCE Calculates the epipolar line distances of the
% provided image points.
% 
% d = EPIPOLARLINEDISTANCE(F, p1, p2) calculates the epipolar line distance
% of the homogeneous points p2 (3xN) in the second view. 
% F (3x3) is the fundamental matrix that relates p1, homogeneous points 
% (3xN) in the first view, to p2.
% d is (1xN) and contains the distances of the points in p2 from the
% epipolar line.

arguments
    F (3,3)
    p1 (3,:)
    p2 {mustBeEqualSize(p1,p2)}
end

l = F * p1; % epipolar line
d = abs(dot(l, p2)) ./ vecnorm(l(1:2,:), 2);

end

function mustBeEqualSize(a,b)
    if ~isequal(size(a),size(b))
        eid = 'Size:notEqual';
        msg = 'Size of first input must equal size of second input.';
        throwAsCaller(MException(eid,msg))
    end
end