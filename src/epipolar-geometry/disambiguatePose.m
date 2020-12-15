function [R, t, landmarks, landmarks_idx] = disambiguatePose(...
    Rots, t, p1, p2, K1, K2, T_1W, verbose)
% DISAMBIGUATEPOSE Given the correspondences, returns the true pose from
% the possible rotation matrices and translation vectors.
%
% [R, t] = DISAMBIGUATEPOSE(Rots, t, p1, p2, K) disambiguates the rotations
% matrices and the translations vectors given the p1 and p2 points
% correspondences and the intrinsics camera matrix shared between the two
% view (e.g., same camera).
% Rots is 3x3xL (L should be 4), possible rotation matrices.
% t is 3x1, t and -t are the possible translation vectors.
% p1 is 3xM, homogenous image coordinates in the first image [u; v; 1].
% p2 is 3xM, homogenous image coordinates in the second image [u; v; 1].
% K is 3x3, intrinsic matrix of the first camera.
% Rots and t can be obtain through the decomposeEssentialMatrix function,
% see the example for more.
%
% DISAMBIGUATEPOSE(..., K2) can be used when the two camera views
% has different intrinsic matrices (e.g., different cameras).
%
% [..., landmarks, landmarks_idx] = DISAMBIGUATEPOSE(...) additionally
% returns the 3D position of the triangulate landmarks (the triangulation
% is used to disambiguate the pose.
% landmarks is 4xN, N <= M, homogenous 3D coordinates with respect to the
% first camera frame. N = nnz(landmarks_idx) is the number of valid
% landmarks with the most likely [R, t] pose and landmarks_idx is a logical
% indexing of p0 and p1 to understand which one are inliers with respect
% the the same pose.
%
% DISAMBIGUATEPOSE(..., T_1W) additionally allows to triangulate the points
% with respect to a different frame (default is eye(3,4)).
%
% Example:
% K is the intrinsic matrix of the calibrated camera. p1 and p2 are
% the corresponding matches in two consecutive keyframes.
% E = estimateEssentialMatrix(p1, p2, K);
% [Rots,t] = decomposeEssentialMatrix(E);
% [R,t] = disambiguatePose(Rots,t,p0,p1,K);
% R is the most likely rotation matrix and t the most likely translation
% vector. 
% T_21 = [R t; 0 0 0 1] is the transformation from frame 1 to frame 2.
%
% See also DECOMPOSEESSENTIALMATRIX, LINEARTRIANGULATION,
% TRIANGULATEFROMPOSE.

arguments
    Rots (3,3,:)
    t (3,1)
    p1 (3,:)
    p2 {mustBeEqualSize(p1,p2)}
    K1 (3,3)
    K2 (3,3) = K1
    T_1W (3,4) = eye(3,4)
    verbose logical = false;
end

M1 = K1 * T_1W;

R = [];
t__ = t;
t = [];
v = -1;
for rotMatrixIndex = 1 : size(Rots, 3)
   R_ = Rots(:,:,rotMatrixIndex);
   for sign = [-1 1]
       t_ = sign * t__;
       T_21 = [R_ t_; 0 0 0 1];
       M2 = K2 * T_21(1:3,:);
       
       p3d = linearTriangulation(...
           p1, p2, M1, M2); % triangulate wrt camera 1
       p3d_cam2 = T_21 * p3d; % coordinates wrt camera 2

       % A point is valid iff is in front of both cameras
       valid_p3d_cam1 = p3d(3,:) > 0;
       valid_p3d_cam2 = p3d_cam2(3,:) > 0;
       valid_p3d = valid_p3d_cam1 & valid_p3d_cam2;

       v_ = nnz(valid_p3d);
       if v_ > v
           v = v_;
           R = R_;
           t = t_;
           landmarks = p3d(:, valid_p3d);
           landmarks_idx = valid_p3d;
       end
   end
end

%assert(v > 0, 'Could not find a single valid landmark.');
verboseDisp(verbose, ...
    'Number of valid landmarks is %d.\n', v);

end

function mustBeEqualSize(a,b)
    if ~isequal(size(a),size(b))
        eid = 'Size:notEqual';
        msg = 'Size of first input must equal size of second input.';
        throwAsCaller(MException(eid,msg))
    end
end
