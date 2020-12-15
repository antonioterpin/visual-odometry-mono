function landmarks = triangulateFromPose(p1, p2, T_21, K1, K2, T_1W)
%TRIANGULATEFROMPOSE Triangulate landmarks given 2D correspondences.
%
% landmarks = TRIANGULATEFROMPOSE(p1,p2,T_21,K)
% p1 and p2 are 3xN, the 2D homogenous coordinates of the correspondences in the
% first and second images [u; v; 1].
% T_21 is 4x4, the homogenous transformation matrix from the first camera
% frame to the second.
% K is 3x3, the intrinsic matrix of the first camera view.
%
% TRIANGULATEFROMPOSE(...,K2) additionally allows for different intrinsics.
% K2 is 3x3, the intrinsic matrix of the second camera view.
%
% TRIANGULATEFROMPOSE(...,T_1W) additionally allows to obtain the landmarks
% relative to the world frame.
% T_1W is 4x4, the homogenous transformation matrix from the world frame to
% the camera frame.

arguments
    p1 (3,:)
    p2 {mustBeEqualSize(p1,p2)}
    T_21 (4,4)
    K1 (3,3)
    K2 (3,3) = K1
    T_1W (4,4) = eye(4)
end

M1 = K1 * T_1W(1:3,:);
M2 = K2 * T_21(1:3,:) * T_1W;

landmarks = linearTriangulation(p1,p2,M1,M2);

end

function mustBeEqualSize(a,b)
    if ~isequal(size(a),size(b))
        eid = 'Size:notEqual';
        msg = 'Size of first input must equal size of second input.';
        throwAsCaller(MException(eid,msg))
    end
end
